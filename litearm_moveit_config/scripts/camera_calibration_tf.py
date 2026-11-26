#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""camera_calibration_tf.py — 把手眼标定结果接进真机的 TF 树。

标定文件里的 ``T_base_cam`` 约定是 ``T_A_B``：把 B 系的点变换到 A 系，
所以它是相机在机器人基座坐标系中的位姿。标定通常是在相机的某个光学帧
（D435i 彩色流是 ``camera_color_optical_frame``）下做的，而 RealSense
驱动自己会发布：

    camera_link -> camera_color_optical_frame      (出厂外参，静态)
    camera_link -> camera_depth_optical_frame      (同上；点云用这个帧)

本节点只补最上面一段，发布：

    base_link -> camera_link

计算方式：

    T_base_camera_link = T_base_calib @ inv(T_camera_link_calib)

其中 ``T_camera_link_calib`` 从驱动的 /tf_static 里实时查。这样：
  * 不会把 ``base_link -> camera_*_optical_frame`` 直接接进树里（那会让
    光学帧有两个父节点，和驱动冲突）；
  * 标定换到彩色帧或深度帧，只改 ``calibration_frame``，不用改代码；
  * 驱动内部的外参（深度/彩色光心之间约 1.5 cm 的偏移）由驱动负责。

参数
====
    calibration_file   标定 JSON 路径（含 transforms.T_base_cam.matrix）
    calibration_frame  标定结果对应的相机帧，默认 camera_color_optical_frame
    base_frame         机器人基座帧，默认 base_link
    camera_link_frame  驱动的机体帧，默认 camera_link
    tf_timeout         等待驱动 TF 的提示时间（s）；超时后继续重试并每 5 s 报一次

诊断
====
启动后会打印标定质量、T_base_calib、驱动外参和最终发布的 TF。真机上
先用 ``ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame``
确认整条链存在，再让 supervisor 接收目标。
"""

import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


# ── 4×4 齐次变换小工具（不引入 numpy，保持脚本轻量）────────────────────

def _identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _inverse(t):
    """刚体变换的逆：R^T, -R^T t。"""
    r = [[t[i][j] for j in range(3)] for i in range(3)]
    p = [t[i][3] for i in range(3)]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    pt = [-sum(rt[i][j] * p[j] for j in range(3)) for i in range(3)]
    out = _identity()
    for i in range(3):
        for j in range(3):
            out[i][j] = rt[i][j]
        out[i][3] = pt[i]
    return out


def _quat_to_matrix(q):
    x, y, z, w = [float(v) for v in q]
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return _identity()[:3][:3]
    s = 2.0 / n
    return [
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ]


def _matrix_to_quat(r):
    """旋转矩阵 -> 四元数 (x, y, z, w)，数值稳定分支。"""
    m00, m01, m02 = r[0]
    m10, m11, m12 = r[1]
    m20, m21, m22 = r[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def _orthonormalize(t):
    """检查旋转块；偏差大时用 Gram-Schmidt 修正并返回是否修正过。"""
    r = [[t[i][j] for j in range(3)] for i in range(3)]
    det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
           - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
           + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    orth_err = max(abs(sum(r[k][i] * r[k][j] for k in range(3))
                        - (1.0 if i == j else 0.0))
                   for i in range(3) for j in range(3))
    if abs(det - 1.0) < 1e-3 and orth_err < 1e-3:
        return t, False

    def dot(a, b):
        return sum(a[k] * b[k] for k in range(3))

    def norm(v):
        return math.sqrt(dot(v, v))

    c0 = [r[i][0] for i in range(3)]
    c1 = [r[i][1] for i in range(3)]
    n0 = norm(c0)
    if n0 < 1e-9:
        raise ValueError("T_base_cam 的旋转块退化（第一列为零）")
    c0 = [v / n0 for v in c0]
    c1 = [c1[i] - dot(c1, c0) * c0[i] for i in range(3)]
    n1 = norm(c1)
    if n1 < 1e-9:
        raise ValueError("T_base_cam 的旋转块退化（前两列共线）")
    c1 = [v / n1 for v in c1]
    c2 = [c0[1] * c1[2] - c0[2] * c1[1],
          c0[2] * c1[0] - c0[0] * c1[2],
          c0[0] * c1[1] - c0[1] * c1[0]]
    out = [row[:] for row in t]
    for i in range(3):
        out[i][0], out[i][1], out[i][2] = c0[i], c1[i], c2[i]
    return out, True


def _transform_to_matrix(transform):
    out = _identity()
    r = _quat_to_matrix([transform.rotation.x, transform.rotation.y,
                         transform.rotation.z, transform.rotation.w])
    for i in range(3):
        for j in range(3):
            out[i][j] = r[i][j]
        out[i][3] = [transform.translation.x,
                     transform.translation.y,
                     transform.translation.z][i]
    return out


def _load_calibration(path):
    with open(path, "r", encoding="utf-8") as handle:
        doc = json.load(handle)
    try:
        matrix = doc["transforms"]["T_base_cam"]["matrix"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "标定文件缺少 transforms.T_base_cam.matrix；"
            "需要 HandEye Studio 导出的 JSON") from exc
    if (not isinstance(matrix, list) or len(matrix) != 4
            or any(len(row) != 4 for row in matrix)):
        raise ValueError("T_base_cam 不是 4×4 矩阵")
    t = [[float(v) for v in row] for row in matrix]
    if not all(math.isfinite(v) for row in t for v in row):
        raise ValueError("T_base_cam 含非有限值")
    t, fixed = _orthonormalize(t)
    return t, doc, fixed


class CameraCalibrationTf(Node):
    def __init__(self):
        super().__init__("camera_calibration_tf")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("calibration_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("tf_timeout", 10.0)

        path = self.get_parameter("calibration_file").value
        if not path:
            raise RuntimeError("参数 calibration_file 不能为空")
        path = os.path.abspath(os.path.expanduser(path))
        self.calibration_frame = str(
            self.get_parameter("calibration_frame").value).strip()
        self.base_frame = str(self.get_parameter("base_frame").value).strip()
        self.camera_link_frame = str(
            self.get_parameter("camera_link_frame").value).strip()
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)

        try:
            self.T_base_calib, self.doc, self.fixed = _load_calibration(path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"无法读取标定文件 {path}: {exc}") from exc

        self.get_logger().info(f"标定文件: {path}")
        self.get_logger().info(
            f"T_{self.base_frame}_{self.calibration_frame} 平移 = "
            f"{[round(v, 4) for v in (self.T_base_calib[0][3], self.T_base_calib[1][3], self.T_base_calib[2][3])]} m")
        self._log_quality()
        if self.fixed:
            self.get_logger().warn(
                "T_base_cam 的旋转块不正交，已做 Gram-Schmidt 修正；"
                "请确认标定文件没有被手工改过")

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.published = False
        self.start_time = self.get_clock().now()
        self.last_warn = self.start_time
        self.timer = self.create_timer(0.2, self._try_publish)

    def _log_quality(self):
        quality = self.doc.get("quality", "未知")
        metrics = self.doc.get("metrics", {}) or {}
        median = metrics.get("median_translation_mm")
        worst = metrics.get("max_translation_mm")
        spread = metrics.get("constant_frame_spread_mm")
        text = f"标定质量: {quality}"
        if median is not None:
            text += f", 中位平移 {median:.2f} mm"
        if worst is not None:
            text += f", 最大 {worst:.2f} mm"
        if spread is not None:
            text += f", 常值帧散布 {spread:.2f} mm"
        log = self.get_logger().warn if quality == "偏低" else self.get_logger().info
        log(text)
        camera = self.doc.get("camera", {}) or {}
        k = camera.get("intrinsic_matrix")
        if isinstance(k, list) and len(k) >= 1 and len(k[0]) >= 1:
            self.get_logger().info(
                f"标定内参 fx={float(k[0][0]):.2f}；"
                "真机上应与 /camera/color/camera_info 的 k[0] 一致")

    def _try_publish(self):
        if self.published:
            return
        try:
            if not self.buffer.can_transform(
                    self.camera_link_frame, self.calibration_frame,
                    rclpy.time.Time()):
                self._warn_waiting()
                return
            tf_msg = self.buffer.lookup_transform(
                self.camera_link_frame, self.calibration_frame,
                rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - TF 查询失败时继续重试
            self._warn_waiting(str(exc))
            return

        T_camera_link_calib = _transform_to_matrix(tf_msg.transform)
        T_base_camera_link = _matmul(self.T_base_calib,
                                     _inverse(T_camera_link_calib))
        t = [T_base_camera_link[i][3] for i in range(3)]
        q = _matrix_to_quat(
            [[T_base_camera_link[i][j] for j in range(3)] for i in range(3)])

        msg = TransformStamped()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = self.base_frame
        msg.child_frame_id = self.camera_link_frame
        msg.transform.translation.x = t[0]
        msg.transform.translation.y = t[1]
        msg.transform.translation.z = t[2]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(msg)

        self.get_logger().info(
            f"驱动外参 {self.camera_link_frame} -> {self.calibration_frame}: "
            f"t={[round(v, 4) for v in (T_camera_link_calib[0][3], T_camera_link_calib[1][3], T_camera_link_calib[2][3])]} m")
        self.get_logger().info(
            f"发布静态 TF {self.base_frame} -> {self.camera_link_frame}: "
            f"t={[round(v, 4) for v in t]} m, quat={[round(v, 5) for v in q]}")
        self.published = True
        self.timer.cancel()

    def _warn_waiting(self, detail=""):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < self.tf_timeout:
            return
        if (now - self.last_warn).nanoseconds / 1e9 < 5.0:
            return
        self.last_warn = now
        suffix = f" ({detail})" if detail else ""
        self.get_logger().error(
            f"等待 TF {self.camera_link_frame} -> {self.calibration_frame} "
            f"已超过 {elapsed:.1f} s{suffix}。请确认 RealSense 驱动已启动、"
            f"calibration_frame 正确、/tf_static 已发布。")
        # 继续重试：相机晚启动时节点仍能自动接上。


def main():
    rclpy.init()
    try:
        node = CameraCalibrationTf()
    except Exception as exc:  # noqa: BLE001 - 配置错误转成明确的退出码
        print(f"✗ camera_calibration_tf 启动失败: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
