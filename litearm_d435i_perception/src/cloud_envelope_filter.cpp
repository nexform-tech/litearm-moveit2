// cloud_envelope_filter.cpp — 可切换的机械臂包络点云过滤。
//
// 现有 sensors_3d_real.yaml 的 padding_offset=0.02 已经让 MoveIt 的
// ShapeMask 滤掉碰撞模型周围 2 cm 内的点。本节点做的是“再额外 1 cm”：
// 用同一套机器人碰撞模型，按 padding=0.03 把模型周围 3 cm 内的点从
// 点云里去掉，再交给 Octomap。默认关闭；RViz 里点击交互标记切换。
//
// 为什么单独做节点，而不是改 MoveIt 参数：PointCloudOctomapUpdater 的
// padding_offset 是启动时参数，不能运行时切换；本节点在点云进入
// /camera/depth/color/points_envelope 之前做过滤，点击即可生效。
//
// 过滤在点云的原始相机坐标系里完成：ShapeMask 的每个机器人 link 变换
// 由 base_link 组合到相机系，点云本身不搬来搬去，Octomap 的 sensor
// origin / 射线清理保持正确。

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "interactive_markers/interactive_marker_server.hpp"
#include "moveit/point_containment_filter/shape_mask.h"
#include "moveit/robot_model/link_model.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/menu_entry.hpp"

namespace
{
Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::Transform& t)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() << t.translation.x, t.translation.y, t.translation.z;
  Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  q.normalize();
  out.linear() = q.toRotationMatrix();
  return out;
}
}  // namespace

class CloudEnvelopeFilter : public rclcpp::Node
{
public:
  CloudEnvelopeFilter()
  : Node("cloud_envelope_filter"),
    tf_buffer_(this->get_clock())
  {
    declare_parameter("robot_description", "");
    declare_parameter("robot_description_semantic", "");
    declare_parameter("input_topic", "/camera/depth/color/points");
    declare_parameter("output_topic", "/camera/depth/color/points_envelope");
    declare_parameter("base_frame", "base_link");
    declare_parameter("marker_frame", "base_link");
    declare_parameter("padding", 0.03);
    declare_parameter("enabled", false);
    declare_parameter("joint_timeout", 1.0);
    declare_parameter("max_sensor_dist", 100.0);

    get_parameter("robot_description", robot_description_);
    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("base_frame", base_frame_);
    get_parameter("marker_frame", marker_frame_);
    get_parameter("padding", padding_);
    get_parameter("enabled", enabled_);
    get_parameter("joint_timeout", joint_timeout_);
    get_parameter("max_sensor_dist", max_sensor_dist_);

    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    output_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, qos);
    input_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        cloudCallback(msg);
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        jointCallback(msg);
      });

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "cloud_envelope_filter", this);

    RCLCPP_INFO(
      get_logger(),
      "cloud_envelope_filter: in=%s out=%s frame=%s padding=%.3f m "
      "enabled=%s",
      input_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str(),
      padding_, enabled_ ? "true" : "false");
  }

  // RobotModel 需要 shared_from_this()，所以在 make_shared 之后单独调用。
  void init()
  {
    try {
      robot_model_loader::RobotModelLoader::Options options("robot_description");
      options.load_kinematics_solvers_ = false;
      robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(
        shared_from_this(), options);
      robot_model_ = robot_model_loader_->getModel();
    } catch (const std::exception& exc) {
      RCLCPP_ERROR(get_logger(), "加载机器人模型失败: %s", exc.what());
      return;
    }
    if (!robot_model_) {
      RCLCPP_ERROR(get_logger(), "机器人模型为空；过滤节点将直通点云");
      return;
    }

    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      tf_buffer_, shared_from_this(), false);

    robot_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    shape_mask_ = std::make_unique<point_containment_filter::ShapeMask>(
      [this](point_containment_filter::ShapeHandle handle,
             Eigen::Isometry3d& transform) {
        return shapeTransform(handle, transform);
      });

    try {
      for (const moveit::core::LinkModel* link :
        robot_model_->getLinkModelsWithCollisionGeometry())
      {
        const auto& shapes = link->getShapes();
        const auto& origins = link->getCollisionOriginTransforms();
        for (size_t i = 0; i < shapes.size(); ++i) {
          if (!shapes[i]) {
            continue;
          }
          const auto handle = shape_mask_->addShape(shapes[i], 1.0, padding_);
          ShapeInfo info;
          info.link_name = link->getName();
          info.origin = i < origins.size() ? origins[i] :
            Eigen::Isometry3d::Identity();
          shape_infos_[handle] = info;
          link_names_.insert(info.link_name);
        }
      }
    } catch (const std::exception& exc) {
      RCLCPP_ERROR(
        get_logger(), "构建 ShapeMask 失败，过滤节点将直通点云: %s",
        exc.what());
      shape_mask_.reset();
      shape_infos_.clear();
      link_names_.clear();
      return;
    }

    RCLCPP_INFO(
      get_logger(), "包络过滤模型就绪: %zu 个碰撞体, %zu 个 link, padding=%.3f m",
      shape_infos_.size(), link_names_.size(), padding_);
    makeMarker();
  }

private:
  struct ShapeInfo
  {
    std::string link_name;
    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  };

  void jointCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
  {
    if (!robot_state_ || msg->name.size() != msg->position.size()) {
      return;
    }
    try {
      robot_state_->setVariablePositions(msg->name, msg->position);
      robot_state_->updateLinkTransforms();
      last_joint_time_ = now().seconds();
    } catch (const std::exception& exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "应用 /joint_states 到机器人模型失败: %s", exc.what());
    }
  }

  bool shapeTransform(
    point_containment_filter::ShapeHandle handle,
    Eigen::Isometry3d& transform) const
  {
    const auto info = shape_infos_.find(handle);
    if (info == shape_infos_.end()) {
      return false;
    }
    const auto link = current_link_transforms_.find(info->second.link_name);
    if (link == current_link_transforms_.end()) {
      return false;
    }
    transform = link->second * info->second.origin;
    return true;
  }

  bool updateLinkTransforms(const std::string& cloud_frame)
  {
    if (!robot_state_ || !robot_model_) {
      return false;
    }
    // 相机是固定外置的，base_link -> camera 是静态 TF，用最新值即可；
    // 不用带超时的 lookup，否则单线程 executor 里点云回调会阻塞 TF 回调。
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        cloud_frame, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "找不到 %s -> %s 的 TF，包络过滤直通: %s",
        cloud_frame.c_str(), base_frame_.c_str(), exc.what());
      return false;
    }

    const Eigen::Isometry3d T_cloud_base = transformToEigen(tf.transform);
    current_link_transforms_.clear();
    for (const std::string& link_name : link_names_) {
      const Eigen::Isometry3d& T_base_link =
        robot_state_->getGlobalLinkTransform(link_name);
      current_link_transforms_[link_name] = T_cloud_base * T_base_link;
    }
    return true;
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
  {
    if (!enabled_ || !shape_mask_ || !robot_state_) {
      output_pub_->publish(*cloud);
      return;
    }
    if (last_joint_time_ < 0.0 ||
      now().seconds() - last_joint_time_ > joint_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "没有新的 /joint_states，包络过滤直通");
      output_pub_->publish(*cloud);
      return;
    }
    if (!updateLinkTransforms(cloud->header.frame_id)) {
      output_pub_->publish(*cloud);
      return;
    }

    std::vector<int> mask;
    try {
      shape_mask_->maskContainment(
        *cloud, Eigen::Vector3d::Zero(), 0.0, max_sensor_dist_, mask);
    } catch (const std::exception& exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "ShapeMask 计算失败，包络过滤直通: %s", exc.what());
      output_pub_->publish(*cloud);
      return;
    }

    const size_t expected = static_cast<size_t>(cloud->width) * cloud->height;
    if (mask.size() != expected) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "ShapeMask 长度 %zu != 点数 %zu，包络过滤直通",
        mask.size(), expected);
      output_pub_->publish(*cloud);
      return;
    }

    auto filtered = std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
    filtered->is_dense = false;
    try {
      sensor_msgs::PointCloud2Iterator<float> ix(*filtered, "x");
      sensor_msgs::PointCloud2Iterator<float> iy(*filtered, "y");
      sensor_msgs::PointCloud2Iterator<float> iz(*filtered, "z");
      const float nan = std::numeric_limits<float>::quiet_NaN();
      for (size_t i = 0; i < mask.size(); ++i, ++ix, ++iy, ++iz) {
        if (mask[i] == point_containment_filter::ShapeMask::INSIDE) {
          *ix = nan;
          *iy = nan;
          *iz = nan;
        }
      }
    } catch (const std::exception& exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "点云没有 x/y/z 字段，包络过滤直通: %s", exc.what());
      output_pub_->publish(*cloud);
      return;
    }
    output_pub_->publish(*filtered);
  }

  void makeMarker()
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = marker_frame_;
    marker.header.stamp = now();
    marker.pose.orientation.w = 1.0;
    marker.pose.position.z = 0.38;
    marker.scale = 0.25;
    std::ostringstream pad_text;
    pad_text << static_cast<int>(std::lround(padding_ * 100.0)) << "cm";
    const std::string pad = pad_text.str();

    marker.name = "cloud_envelope_filter";
    marker.description = enabled_
      ? pad + " 包络过滤: 开 (点击关闭)"
      : pad + " 包络过滤: 关 (点击开启)";

    visualization_msgs::msg::MenuEntry toggle;
    toggle.id = 1;
    toggle.parent_id = 0;
    toggle.title = enabled_
      ? "关闭 " + pad + " 包络过滤（仅保留内置 2cm）"
      : "开启 " + pad + " 包络过滤";
    toggle.command_type = visualization_msgs::msg::MenuEntry::FEEDBACK;
    toggle.command = "toggle";
    marker.menu_entries.push_back(toggle);

    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
    sphere.color.a = 0.9;
    if (enabled_) {
      sphere.color.r = 0.1;
      sphere.color.g = 0.85;
      sphere.color.b = 0.2;
    } else {
      sphere.color.r = 0.55;
      sphere.color.g = 0.55;
      sphere.color.b = 0.55;
    }

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = "toggle";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;
    control.description = "点击切换 " + pad + " 包络过滤";
    control.markers.push_back(sphere);
    marker.controls.push_back(control);

    marker_server_->insert(
      marker,
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& event) {
        markerFeedback(event);
      });
    marker_server_->applyChanges();
  }

  void markerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& event)
  {
    const bool menu_toggle =
      event->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT &&
      event->menu_entry_id == 1;
    const bool button_toggle =
      event->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
    if (!menu_toggle && !button_toggle) {
      return;
    }
    enabled_ = !enabled_;
    RCLCPP_INFO(
      get_logger(), "%.0fcm 包络过滤: %s", padding_ * 100.0,
      enabled_ ? "开" : "关");
    makeMarker();
  }

  std::string robot_description_;
  std::string input_topic_;
  std::string output_topic_;
  std::string base_frame_;
  std::string marker_frame_;
  double padding_ = 0.03;
  bool enabled_ = false;
  double joint_timeout_ = 1.0;
  double max_sensor_dist_ = 100.0;
  double last_joint_time_ = -1.0;

  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<moveit::core::RobotState> robot_state_;
  std::unique_ptr<point_containment_filter::ShapeMask> shape_mask_;
  std::map<point_containment_filter::ShapeHandle, ShapeInfo> shape_infos_;
  std::set<std::string> link_names_;
  std::map<std::string, Eigen::Isometry3d> current_link_transforms_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudEnvelopeFilter>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
