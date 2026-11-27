// cloud_envelope_filter.cpp — 可切换的机械臂包络点云过滤。
//
// 现有 sensors_3d_real.yaml 的 padding_offset=0.02 已经让 MoveIt 的
// ShapeMask 滤掉碰撞模型周围 2 cm 内的点。本节点做的是“再额外 1 cm”：
// 用同一套机器人碰撞模型，按 padding=0.03 把模型周围 3 cm 内的点从
// 点云里去掉，再交给 Octomap。默认关闭；RViz 里点击交互标记切换。
//
// 另外本节点始终做一步“飞点过滤”（flying-pixel filter，可用
// flying_pixel_filter:=false 关闭）：
//   * D435i 在深度不连续处（机械臂轮廓 vs 桌面/背景）会输出一片介于
//     前景和背景之间的中间深度，形成从机械臂边缘扇出去的一层薄面；
//   * ShapeMask 只删除落在碰撞模型内部的点，模型外这层扇面会原样进入
//     Octomap，在手臂离开后留下台阶状幽灵障碍；
//   * 用 /camera/depth/camera_info 把每个点投影回像素平面，按 2D 邻域
//     的最大/最小深度差判断：跨度过大的像素（包括真实边缘的 1~2 像素）
//     置 NaN。代价是障碍物轮廓少一圈像素，换取不再产生永久幽灵。
//   * 点云不必是有序的：realsense 的 /camera/depth/color/points 是
//     width=N,height=1 的铺平点云，投影后仍与深度像素一一对应。
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
#include <cstdint>
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
#include "sensor_msgs/msg/camera_info.hpp"
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
    declare_parameter("flying_pixel_filter", true);
    declare_parameter("flying_pixel_depth_jump", 0.03);
    declare_parameter("flying_pixel_radius", 1);
    declare_parameter("camera_info_topic", "/camera/depth/camera_info");

    get_parameter("robot_description", robot_description_);
    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("base_frame", base_frame_);
    get_parameter("marker_frame", marker_frame_);
    get_parameter("padding", padding_);
    get_parameter("enabled", enabled_);
    get_parameter("joint_timeout", joint_timeout_);
    get_parameter("max_sensor_dist", max_sensor_dist_);
    get_parameter("flying_pixel_filter", flying_pixel_filter_);
    get_parameter("flying_pixel_depth_jump", flying_pixel_depth_jump_);
    get_parameter("flying_pixel_radius", flying_pixel_radius_);
    get_parameter("camera_info_topic", camera_info_topic_);
    flying_pixel_radius_ = std::clamp(flying_pixel_radius_, 0, 8);
    if (!std::isfinite(flying_pixel_depth_jump_) ||
      flying_pixel_depth_jump_ <= 0.0)
    {
      flying_pixel_depth_jump_ = 0.03;
    }

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
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        cameraInfoCallback(msg);
      });

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "cloud_envelope_filter", this);

    RCLCPP_INFO(
      get_logger(),
      "cloud_envelope_filter: in=%s out=%s frame=%s padding=%.3f m "
      "enabled=%s flying_pixel=%s (jump=%.0f mm, radius=%d px, info=%s)",
      input_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str(),
      padding_, enabled_ ? "true" : "false",
      flying_pixel_filter_ ? "true" : "false",
      flying_pixel_depth_jump_ * 1000.0, flying_pixel_radius_,
      camera_info_topic_.c_str());
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

  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
  {
    if (msg->width == 0 || msg->height == 0) {
      return;
    }
    const double fx = msg->k[0];
    const double fy = msg->k[4];
    const double cx = msg->k[2];
    const double cy = msg->k[5];
    if (!(std::isfinite(fx) && std::isfinite(fy) && fx > 0.0 && fy > 0.0)) {
      return;
    }
    camera_fx_ = fx;
    camera_fy_ = fy;
    camera_cx_ = cx;
    camera_cy_ = cy;
    camera_width_ = msg->width;
    camera_height_ = msg->height;
    has_camera_info_ = true;
  }

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

  // 去掉深度不连续处的飞点：把每个点按深度相机内参投影回像素平面，
  // 建一张 z-buffer；对每个像素看 (2r+1)² 邻域内有效深度的最大/最小差，
  // 跨度超过阈值的像素置无效，再把它上面的点从点云里删掉。飞点本身位于
  // 前景和背景之间，邻域里必然同时包含两端，所以会被整个削掉。
  // 点云不必是有序的：D435i 的 /camera/depth/color/points 是
  // width=N,height=1 的铺平点云，投影后与 /camera/depth/camera_info
  // 的像素一一对应（实测 318417 个点全部落在 848x480 内且无重复）。
  // 返回 true 表示点云被修改过。
  bool applyFlyingPixelFilter(sensor_msgs::msg::PointCloud2& cloud)
  {
    if (!flying_pixel_filter_) {
      return false;
    }
    if (!has_camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "还没有相机内参（%s），飞点过滤跳过",
        camera_info_topic_.c_str());
      return false;
    }
    const size_t width = camera_width_;
    const size_t height = camera_height_;
    const size_t pixels = width * height;
    const size_t count = static_cast<size_t>(cloud.width) * cloud.height;
    if (pixels == 0 || count == 0) {
      return false;
    }

    std::vector<float> zbuffer(pixels, std::numeric_limits<float>::quiet_NaN());
    size_t projected = 0;
    size_t out_of_bounds = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");
      for (size_t i = 0; i < count; ++i, ++ix, ++iy, ++iz) {
        const float z = *iz;
        if (!std::isfinite(z) || z <= 0.0f) {
          continue;
        }
        const float px = (camera_fx_ * (*ix)) / z + camera_cx_;
        const float py = (camera_fy_ * (*iy)) / z + camera_cy_;
        if (px < 0.0f || py < 0.0f) {
          ++out_of_bounds;
          continue;
        }
        const size_t u = static_cast<size_t>(std::lround(px));
        const size_t v = static_cast<size_t>(std::lround(py));
        if (u >= width || v >= height) {
          ++out_of_bounds;
          continue;
        }
        ++projected;
        float& cell = zbuffer[v * width + u];
        if (!std::isfinite(cell) || z < cell) {
          cell = z;
        }
      }
    } catch (const std::exception& exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "点云没有 x/y/z 字段，飞点过滤跳过: %s", exc.what());
      return false;
    }

    if (projected == 0) {
      return false;
    }
    // 内参和点云不匹配时（例如拿彩色内参投深度点云）大量点会跑到画幅外；
    // 这时宁可直通，也不要用错误的像素邻域把好点删掉。
    if (out_of_bounds > projected / 10) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "飞点过滤跳过：%zu/%zu 个点投影到画幅外，检查 %s 是否与点云匹配",
        out_of_bounds, out_of_bounds + projected,
        camera_info_topic_.c_str());
      return false;
    }

    const int radius = flying_pixel_radius_;
    std::vector<uint8_t> drop(pixels, 0);
    size_t dropped = 0;
    for (size_t y = 0; y < height; ++y) {
      const size_t y0 = y > static_cast<size_t>(radius) ?
        y - static_cast<size_t>(radius) : 0;
      const size_t y1 = std::min(height - 1, y + static_cast<size_t>(radius));
      for (size_t x = 0; x < width; ++x) {
        const size_t index = y * width + x;
        const float center = zbuffer[index];
        if (!std::isfinite(center)) {
          continue;
        }
        const size_t x0 = x > static_cast<size_t>(radius) ?
          x - static_cast<size_t>(radius) : 0;
        const size_t x1 = std::min(width - 1, x + static_cast<size_t>(radius));
        float lo = center;
        float hi = center;
        for (size_t ny = y0; ny <= y1; ++ny) {
          const size_t row = ny * width;
          for (size_t nx = x0; nx <= x1; ++nx) {
            const float value = zbuffer[row + nx];
            if (!std::isfinite(value)) {
              continue;
            }
            lo = std::min(lo, value);
            hi = std::max(hi, value);
          }
        }
        // 实心表面在邻域内的深度变化通常只有几毫米；超过阈值就是
        // 前景/背景交界，整条交界带（含真实边缘 1~2 像素）一起丢弃。
        if (hi - lo > flying_pixel_depth_jump_) {
          drop[index] = 1;
          ++dropped;
        }
      }
    }

    if (dropped == 0) {
      return false;
    }

    size_t removed = 0;
    try {
      sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
      const float nan = std::numeric_limits<float>::quiet_NaN();
      for (size_t i = 0; i < count; ++i, ++ix, ++iy, ++iz) {
        const float z = *iz;
        if (!std::isfinite(z) || z <= 0.0f) {
          continue;
        }
        const float px = (camera_fx_ * (*ix)) / z + camera_cx_;
        const float py = (camera_fy_ * (*iy)) / z + camera_cy_;
        if (px < 0.0f || py < 0.0f) {
          continue;
        }
        const size_t u = static_cast<size_t>(std::lround(px));
        const size_t v = static_cast<size_t>(std::lround(py));
        if (u >= width || v >= height || !drop[v * width + u]) {
          continue;
        }
        *ix = nan;
        *iy = nan;
        *iz = nan;
        ++removed;
      }
    } catch (const std::exception& exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "点云没有 x/y/z 字段，飞点过滤跳过: %s", exc.what());
      return false;
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "飞点过滤: 删除 %zu/%zu 点（%zu 个点投影越界，邻域跨度 > %.0f mm，半径 %d px）",
      removed, count, out_of_bounds, flying_pixel_depth_jump_ * 1000.0, radius);
    return true;
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
  {
    std::shared_ptr<sensor_msgs::msg::PointCloud2> filtered;
    bool changed = false;

    // 飞点过滤始终执行，与 RViz 里的包络开关无关：幽灵障碍主要来自
    // 模型外的边缘飞点，而不是 2/3 cm 包络本身。
    if (flying_pixel_filter_) {
      filtered = std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
      changed = applyFlyingPixelFilter(*filtered);
    }

    if (enabled_) {
      if (!shape_mask_ || !robot_state_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "包络过滤不可用（机器人模型未就绪），直通");
      } else if (last_joint_time_ < 0.0 ||
        now().seconds() - last_joint_time_ > joint_timeout_)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "没有新的 /joint_states，包络过滤直通");
      } else if (updateLinkTransforms(cloud->header.frame_id)) {
        if (!filtered) {
          filtered = std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
        }
        std::vector<int> mask;
        bool mask_ok = true;
        try {
          shape_mask_->maskContainment(
            *filtered, Eigen::Vector3d::Zero(), 0.0, max_sensor_dist_, mask);
        } catch (const std::exception& exc) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "ShapeMask 计算失败，包络过滤直通: %s", exc.what());
          mask_ok = false;
        }

        const size_t expected =
          static_cast<size_t>(filtered->width) * filtered->height;
        if (mask_ok && mask.size() != expected) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "ShapeMask 长度 %zu != 点数 %zu，包络过滤直通",
            mask.size(), expected);
          mask_ok = false;
        }
        if (mask_ok) {
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
            changed = true;
          } catch (const std::exception& exc) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "点云没有 x/y/z 字段，包络过滤直通: %s", exc.what());
          }
        }
      }
    }

    if (changed && filtered) {
      output_pub_->publish(*filtered);
    } else {
      output_pub_->publish(*cloud);
    }
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
  bool flying_pixel_filter_ = true;
  double flying_pixel_depth_jump_ = 0.03;
  int flying_pixel_radius_ = 1;
  std::string camera_info_topic_ = "/camera/depth/camera_info";
  bool has_camera_info_ = false;
  double camera_fx_ = 0.0;
  double camera_fy_ = 0.0;
  double camera_cx_ = 0.0;
  double camera_cy_ = 0.0;
  size_t camera_width_ = 0;
  size_t camera_height_ = 0;

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
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
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
