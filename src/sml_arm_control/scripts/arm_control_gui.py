#!/usr/bin/env python3
"""Industrial/Open-Source style Teach Pendant & Control Studio GUI for sml_arm."""

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Allow importing sibling arm_kinematics.py regardless of working directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_kinematics import (  # noqa: E402
    ARM_JOINTS,
    JOINT_LIMITS,
    POSES,
    clamp_joints,
    compute_fk_local,
    compute_ik_local,
    duration_from_seconds,
    matrix_to_quat,
    matrix_to_rpy,
    quat_to_matrix,
    rpy_to_matrix,
)


DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #11161d;
    color: #e2e8f0;
    font-family: "Segoe UI", "Ubuntu", "DejaVu Sans", sans-serif;
    font-size: 12px;
}
QGroupBox {
    background-color: #18202a;
    border: 1px solid #2b3848;
    border-radius: 6px;
    margin-top: 12px;
    padding: 10px 8px 8px 8px;
    font-weight: bold;
    color: #7dd3fc;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
}
QPushButton {
    background-color: #24303f;
    border: 1px solid #38495e;
    border-radius: 4px;
    padding: 5px 10px;
    color: #f1f5f9;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #2f4054;
    border-color: #38bdf8;
}
QPushButton:pressed {
    background-color: #1c2734;
}
QPushButton:disabled {
    background-color: #192028;
    color: #64748b;
    border-color: #252f3b;
}
QPushButton#btnPrimary {
    background-color: #0284c7;
    border-color: #38bdf8;
    color: #ffffff;
}
QPushButton#btnPrimary:hover {
    background-color: #0369a1;
}
QPushButton#btnMoveIt {
    background-color: #0d9488;
    border-color: #2dd4bf;
    color: #ffffff;
}
QPushButton#btnMoveIt:hover {
    background-color: #0f766e;
}
QPushButton#btnEstop {
    background-color: #dc2626;
    border: 2px solid #fca5a5;
    border-radius: 6px;
    color: #ffffff;
    font-size: 13px;
    font-weight: bold;
    padding: 6px 16px;
}
QPushButton#btnEstop:hover {
    background-color: #b91c1c;
}
QDoubleSpinBox, QSpinBox, QComboBox, QLineEdit {
    background-color: #0d1218;
    border: 1px solid #334155;
    border-radius: 4px;
    padding: 3px 6px;
    color: #f8fafc;
}
QSlider::groove:horizontal {
    height: 6px;
    background: #0d1218;
    border: 1px solid #334155;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #38bdf8;
    border: 1px solid #e0f2fe;
    width: 14px;
    margin: -5px 0;
    border-radius: 7px;
}
QSlider::sub-page:horizontal {
    background: #0284c7;
    border-radius: 3px;
}
QTableWidget {
    background-color: #0d1218;
    border: 1px solid #2b3848;
    gridline-color: #222d3a;
    selection-background-color: #0369a1;
}
QHeaderView::section {
    background-color: #1e2835;
    color: #93c5fd;
    padding: 4px;
    border: 1px solid #2b3848;
    font-weight: bold;
}
QPlainTextEdit {
    background-color: #0b0f14;
    border: 1px solid #2b3848;
    color: #cbd5e1;
    font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace;
    font-size: 11px;
}
QTabWidget::pane {
    border: 1px solid #2b3848;
    border-radius: 4px;
    background-color: #141b23;
}
QTabBar::tab {
    background-color: #1a232e;
    border: 1px solid #2b3848;
    padding: 6px 14px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    color: #94a3b8;
    font-weight: 600;
}
QTabBar::tab:selected {
    background-color: #0284c7;
    color: #ffffff;
}
"""


class Arm3DViewport(QtWidgets.QWidget):
    """Interactive 3D kinematic chain viewport with orbit, zoom, actual/ghost arm, and EE trail."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(360, 320)
        self._azimuth = math.radians(42.0)
        self._elevation = math.radians(24.0)
        self._zoom = 340.0
        self._last_pos: Optional[QtCore.QPoint] = None

        self._actual_joints: List[float] = list(POSES['home'])
        self._target_joints: List[float] = list(POSES['home'])
        self._actual_gripper: float = 0.038
        self._target_gripper: float = 0.038
        self._ee_trail: List[Tuple[float, float, float]] = []

    def reset_view(self) -> None:
        self._azimuth = math.radians(42.0)
        self._elevation = math.radians(24.0)
        self._zoom = 340.0
        self.update()

    def clear_trail(self) -> None:
        self._ee_trail.clear()
        self.update()

    def set_actual_joints(self, joints: Sequence[float]) -> None:
        self._actual_joints = list(joints)
        fk = compute_fk_local(self._actual_joints, frame='world')
        pt = fk['xyz']
        if not self._ee_trail or math.dist(pt, self._ee_trail[-1]) > 0.004:
            self._ee_trail.append(pt)
            if len(self._ee_trail) > 250:
                self._ee_trail.pop(0)
        self.update()

    def set_target_joints(self, joints: Sequence[float]) -> None:
        self._target_joints = list(joints)
        self.update()

    def set_gripper_width(self, actual_m: Optional[float] = None, target_m: Optional[float] = None) -> None:
        if actual_m is not None:
            self._actual_gripper = float(actual_m)
        if target_m is not None:
            self._target_gripper = float(target_m)
        self.update()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self._last_pos = event.pos()

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._last_pos is not None and (event.buttons() & QtCore.Qt.LeftButton):
            dx = event.x() - self._last_pos.x()
            dy = event.y() - self._last_pos.y()
            self._azimuth -= dx * 0.01
            self._elevation = max(-1.35, min(1.45, self._elevation + dy * 0.01))
            self._last_pos = event.pos()
            self.update()

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        delta = event.angleDelta().y()
        factor = 1.1 if delta > 0 else 0.9
        self._zoom = max(120.0, min(900.0, self._zoom * factor))
        self.update()

    def _project(self, pt: Sequence[float]) -> Tuple[float, float, float]:
        """Project world 3D point (x, y, z) to screen coordinates (sx, sy, depth)."""
        # Center view vertically around z = 0.42 m
        x, y, z = float(pt[0]), float(pt[1]), float(pt[2]) - 0.42
        ca, sa = math.cos(self._azimuth), math.sin(self._azimuth)
        ce, se = math.cos(self._elevation), math.sin(self._elevation)

        # Rotate around Z (azimuth)
        x1 = ca * x + sa * y
        y1 = -sa * x + ca * y
        z1 = z

        # Rotate around screen horizontal axis (elevation)
        y2 = ce * y1 - se * z1
        z2 = se * y1 + ce * z1

        cx = self.width() * 0.5
        cy = self.height() * 0.54
        persp = 2.6 / max(0.6, 2.6 - y2)
        sx = cx + x1 * self._zoom * persp
        sy = cy - z2 * self._zoom * persp
        return sx, sy, y2

    def _draw_box_3d(
        self,
        painter: QtGui.QPainter,
        center: Tuple[float, float, float],
        size: Tuple[float, float, float],
        color: QtGui.QColor,
        fill_alpha: int = 45,
    ) -> None:
        cx, cy, cz = center
        hx, hy, hz = size[0] * 0.5, size[1] * 0.5, size[2] * 0.5
        corners_3d = [
            (cx - hx, cy - hy, cz - hz),
            (cx + hx, cy - hy, cz - hz),
            (cx + hx, cy + hy, cz - hz),
            (cx - hx, cy + hy, cz - hz),
            (cx - hx, cy - hy, cz + hz),
            (cx + hx, cy - hy, cz + hz),
            (cx + hx, cy + hy, cz + hz),
            (cx - hx, cy + hy, cz + hz),
        ]
        pts = [self._project(c) for c in corners_3d]
        top_poly = QtGui.QPolygonF([QtCore.QPointF(pts[i][0], pts[i][1]) for i in (4, 5, 6, 7)])
        fill_c = QtGui.QColor(color)
        fill_c.setAlpha(fill_alpha)
        painter.setBrush(QtGui.QBrush(fill_c))
        painter.setPen(QtGui.QPen(color, 1.4))
        painter.drawPolygon(top_poly)
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        for i, j in edges:
            painter.drawLine(QtCore.QPointF(pts[i][0], pts[i][1]), QtCore.QPointF(pts[j][0], pts[j][1]))

    def _draw_chain(
        self,
        painter: QtGui.QPainter,
        joints: Sequence[float],
        gripper_width: float,
        line_color: QtGui.QColor,
        node_color: QtGui.QColor,
        line_width: float,
        dashed: bool = False,
    ) -> Tuple[float, float, float]:
        fk = compute_fk_local(joints, frame='world')
        key_links = [
            'base_link',
            'j1_harmonic_drive',
            'j2_harmonic_drive',
            'j3_harmonic_drive',
            'j4_harmonic_drive',
            'j5_harmonic_drive',
            'j6_harmonic_drive',
            'ee_link',
        ]
        transforms = fk['transforms']
        pts_3d = [transforms[name][:3, 3] for name in key_links]
        projected = [self._project(p) for p in pts_3d]

        pen = QtGui.QPen(line_color, line_width)
        if dashed:
            pen.setStyle(QtCore.Qt.DashLine)
        painter.setPen(pen)

        for i in range(len(projected) - 1):
            p0, p1 = projected[i], projected[i + 1]
            painter.drawLine(QtCore.QPointF(p0[0], p0[1]), QtCore.QPointF(p1[0], p1[1]))

        painter.setBrush(QtGui.QBrush(node_color))
        painter.setPen(QtGui.QPen(QtGui.QColor('#0f172a'), 1.2))
        for idx, (sx, sy, _) in enumerate(projected):
            r = 5.0 if idx in (0, len(projected) - 1) else 3.8
            painter.drawEllipse(QtCore.QPointF(sx, sy), r, r)

        # Draw parallel gripper jaws attached to ee_link
        T_ee = transforms['ee_link']
        p_ee = T_ee[:3, 3]
        R_ee = T_ee[:3, :3]
        jaw_offset = 0.007 + max(0.0, min(0.040, gripper_width))
        left_base = p_ee + R_ee @ np.array([0.0, jaw_offset, 0.024])
        left_tip = p_ee + R_ee @ np.array([0.0, jaw_offset, 0.094])
        right_base = p_ee + R_ee @ np.array([0.0, -jaw_offset, 0.024])
        right_tip = p_ee + R_ee @ np.array([0.0, -jaw_offset, 0.094])
        bar_l = p_ee + R_ee @ np.array([0.0, 0.048, 0.012])
        bar_r = p_ee + R_ee @ np.array([0.0, -0.048, 0.012])

        jaw_color = QtGui.QColor('#f97316') if not dashed else QtGui.QColor(52, 211, 153, 150)
        jaw_pen = QtGui.QPen(jaw_color, 2.6 if not dashed else 1.6)
        if dashed:
            jaw_pen.setStyle(QtCore.Qt.DashLine)
        painter.setPen(jaw_pen)
        for p_a, p_b in ((bar_l, bar_r), (left_base, left_tip), (right_base, right_tip)):
            sa, sb = self._project(p_a), self._project(p_b)
            painter.drawLine(QtCore.QPointF(sa[0], sa[1]), QtCore.QPointF(sb[0], sb[1]))

        # Draw EE orientation triad
        axis_len = 0.07
        for col, col_hex in enumerate(('#ef4444', '#22c55e', '#3b82f6')):
            tip = p_ee + R_ee[:, col] * axis_len
            s0 = self._project(p_ee)
            s1 = self._project(tip)
            c = QtGui.QColor(col_hex)
            if dashed:
                c.setAlpha(130)
            painter.setPen(QtGui.QPen(c, 2.0))
            painter.drawLine(QtCore.QPointF(s0[0], s0[1]), QtCore.QPointF(s1[0], s1[1]))

        return fk['xyz']

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        del event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        # Background gradient
        grad = QtGui.QLinearGradient(0, 0, 0, self.height())
        grad.setColorAt(0.0, QtGui.QColor('#0c1118'))
        grad.setColorAt(1.0, QtGui.QColor('#151e29'))
        painter.fillRect(self.rect(), grad)

        # Draw ground grid on Z = 0.0
        painter.setPen(QtGui.QPen(QtGui.QColor('#223042'), 1.0))
        grid_extent = 0.8
        step = 0.1
        n = int(round(grid_extent / step))
        for k in range(-n, n + 1):
            coord = k * step
            a = self._project((coord, -grid_extent, 0.0))
            b = self._project((coord, grid_extent, 0.0))
            painter.drawLine(QtCore.QPointF(a[0], a[1]), QtCore.QPointF(b[0], b[1]))
            c = self._project((-grid_extent, coord, 0.0))
            d = self._project((grid_extent, coord, 0.0))
            painter.drawLine(QtCore.QPointF(c[0], c[1]), QtCore.QPointF(d[0], d[1]))

        # World origin triad
        o = self._project((0.0, 0.0, 0.0))
        for vec, color_hex in [((0.18, 0.0, 0.0), '#ef4444'), ((0.0, 0.18, 0.0), '#22c55e'), ((0.0, 0.0, 0.18), '#3b82f6')]:
            t = self._project(vec)
            painter.setPen(QtGui.QPen(QtGui.QColor(color_hex), 2.0))
            painter.drawLine(QtCore.QPointF(o[0], o[1]), QtCore.QPointF(t[0], t[1]))

        # Pick & Place Workcell tables and target cube
        self._draw_box_3d(painter, (0.20, -0.32, 0.10), (0.16, 0.16, 0.20), QtGui.QColor('#64748b'), fill_alpha=35)
        self._draw_box_3d(painter, (-0.20, -0.32, 0.10), (0.16, 0.16, 0.20), QtGui.QColor('#14b8a6'), fill_alpha=35)
        self._draw_box_3d(painter, (0.20, -0.32, 0.225), (0.036, 0.036, 0.050), QtGui.QColor('#ef4444'), fill_alpha=90)

        # End-effector trail
        if len(self._ee_trail) >= 2:
            painter.setPen(QtGui.QPen(QtGui.QColor(56, 189, 248, 120), 1.6))
            prev = self._project(self._ee_trail[0])
            for pt in self._ee_trail[1:]:
                cur = self._project(pt)
                painter.drawLine(QtCore.QPointF(prev[0], prev[1]), QtCore.QPointF(cur[0], cur[1]))
                prev = cur

        # Ghost target arm chain
        tgt_xyz = self._draw_chain(
            painter,
            self._target_joints,
            self._target_gripper,
            QtGui.QColor(52, 211, 153, 175),
            QtGui.QColor(16, 185, 129, 200),
            2.4,
            dashed=True,
        )

        # Actual arm chain
        act_xyz = self._draw_chain(
            painter,
            self._actual_joints,
            self._actual_gripper,
            QtGui.QColor('#38bdf8'),
            QtGui.QColor('#fb923c'),
            4.2,
            dashed=False,
        )

        # HUD overlay
        painter.setPen(QtGui.QColor('#94a3b8'))
        painter.setFont(QtGui.QFont('DejaVu Sans Mono', 9))
        painter.drawText(
            12, 20,
            f'ACTUAL EE (m): [{act_xyz[0]:+.3f}, {act_xyz[1]:+.3f}, {act_xyz[2]:+.3f}]'
        )
        painter.setPen(QtGui.QColor('#34d399'))
        painter.drawText(
            12, 36,
            f'TARGET EE (m): [{tgt_xyz[0]:+.3f}, {tgt_xyz[1]:+.3f}, {tgt_xyz[2]:+.3f}]'
        )
        painter.setPen(QtGui.QColor('#64748b'))
        painter.drawText(12, self.height() - 10, 'Drag: Orbit  |  Wheel: Zoom  |  Cyan: Actual  |  Green: Target')


class RosBridgeWorker(QtCore.QObject):
    """Background ROS 2 interface handling JointState, Trajectory Action, and MoveIt 2."""

    joint_states_received = QtCore.pyqtSignal(list, float)
    gripper_state_received = QtCore.pyqtSignal(float)
    connection_status = QtCore.pyqtSignal(bool, bool, bool)
    log_signal = QtCore.pyqtSignal(str)
    fk_verified = QtCore.pyqtSignal(tuple, tuple, tuple, str)
    ik_solved = QtCore.pyqtSignal(bool, list, str)
    motion_finished = QtCore.pyqtSignal(bool, str)

    def __init__(self, ros_args: Optional[List[str]] = None) -> None:
        super().__init__()
        self._ros_args = ros_args or []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._node: Optional[Node] = None
        self._traj_client: Optional[ActionClient] = None
        self._rt_traj_pub = None
        self._gripper_pub = None
        self._moveit_client: Optional[ActionClient] = None
        self._fk_client = None
        self._ik_client = None

        self._active_traj_handle = None
        self._active_moveit_handle = None
        self._last_js_time = 0.0
        self._js_hz = 0.0
        self._latest_joints = list(POSES['home'])

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=self._ros_args)
        self._node = Node('sml_arm_control_gui')
        self._node.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self._rt_traj_pub = self._node.create_publisher(
            JointTrajectory, '/arm_trajectory_controller/joint_trajectory', 5
        )
        self._gripper_pub = self._node.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10
        )
        self._traj_client = ActionClient(
            self._node, FollowJointTrajectory, '/arm_trajectory_controller/follow_joint_trajectory'
        )
        self._moveit_client = ActionClient(self._node, MoveGroup, '/move_action')
        self._fk_client = self._node.create_client(GetPositionFK, '/compute_fk')
        self._ik_client = self._node.create_client(GetPositionIK, '/compute_ik')

        self._running = True
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()
        self.log_signal.emit('ROS 2 bridge started (/joint_states, ros2_control, MoveIt 2).')

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.5)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()

    def _spin_loop(self) -> None:
        last_status_check = 0.0
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.02)
            except Exception:
                break
            now = time.monotonic()
            if now - last_status_check > 0.5:
                last_status_check = now
                js_ok = (now - self._last_js_time) < 2.0
                traj_ok = bool(
                    (self._traj_client and self._traj_client.server_is_ready())
                    or (self._rt_traj_pub and self._rt_traj_pub.get_subscription_count() > 0)
                )
                moveit_ok = bool(
                    self._ik_client and self._ik_client.service_is_ready()
                    and self._moveit_client and self._moveit_client.server_is_ready()
                )
                self.connection_status.emit(js_ok, traj_ok, moveit_ok)

    def _on_joint_state(self, msg: JointState) -> None:
        now = time.monotonic()
        if self._last_js_time > 0.0:
            dt = now - self._last_js_time
            if dt > 1e-4:
                self._js_hz = 0.8 * self._js_hz + 0.2 * (1.0 / dt)
        self._last_js_time = now

        name_map = dict(zip(msg.name, msg.position))
        if all(j in name_map for j in ARM_JOINTS):
            self._latest_joints = [float(name_map[j]) for j in ARM_JOINTS]
            self.joint_states_received.emit(list(self._latest_joints), self._js_hz)
        if 'gripper_left_finger_joint' in name_map:
            self.gripper_state_received.emit(float(name_map['gripper_left_finger_joint']))

    def send_gripper(self, width_m: float) -> None:
        """Command parallel gripper fingers symmetrically to width_m [0.0..0.040] m."""
        if self._gripper_pub is None:
            return
        w = max(0.0, min(0.040, float(width_m)))
        msg = Float64MultiArray()
        msg.data = [w, w]
        self._gripper_pub.publish(msg)
        self.log_signal.emit(f'Gripper command -> {w * 1000.0:.1f} mm/finger')

    def stream_joint_positions_rt(
        self, positions: Sequence[float], reach_time_sec: float = 0.06
    ) -> None:
        """Publish a low-latency JointTrajectory topic message for real-time jogging."""
        if self._rt_traj_pub is None:
            return
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = duration_from_seconds(max(0.04, reach_time_sec))
        traj.points = [pt]
        self._rt_traj_pub.publish(traj)

    def send_joint_trajectory(self, positions: Sequence[float], duration_sec: float) -> None:
        if self._node is None or self._traj_client is None:
            return
        if not self._traj_client.server_is_ready():
            self.log_signal.emit('WARN: /arm_trajectory_controller/follow_joint_trajectory unavailable.')
            self.motion_finished.emit(False, 'Trajectory action server unavailable')
            return

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * 6
        pt.time_from_start = duration_from_seconds(max(0.15, duration_sec))
        traj.points = [pt]
        goal.trajectory = traj

        self.log_signal.emit(
            f'Sending ros2_control trajectory: {[round(p, 3) for p in positions]} ({duration_sec:.2f}s)'
        )
        fut = self._traj_client.send_goal_async(goal)
        fut.add_done_callback(self._on_traj_goal_response)

    def _on_traj_goal_response(self, fut) -> None:
        gh = fut.result()
        if not gh or not gh.accepted:
            self.log_signal.emit('ERROR: Trajectory goal rejected.')
            self.motion_finished.emit(False, 'Trajectory goal rejected')
            return
        self._active_traj_handle = gh
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_traj_result)

    def _on_traj_result(self, fut) -> None:
        self._active_traj_handle = None
        result = fut.result().result
        ok = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        msg = 'Trajectory complete' if ok else f'Trajectory error_code={result.error_code}'
        self.log_signal.emit(msg)
        self.motion_finished.emit(ok, msg)

    def verify_moveit_fk(self, joints: Sequence[float]) -> None:
        if self._fk_client is None or not self._fk_client.service_is_ready():
            self.log_signal.emit('INFO: MoveIt /compute_fk offline; using local FK.')
            return
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = ['ee_link']
        req.robot_state = RobotState()
        req.robot_state.joint_state = JointState(
            name=list(ARM_JOINTS),
            position=[float(j) for j in joints],
        )
        fut = self._fk_client.call_async(req)
        fut.add_done_callback(self._on_fk_response)

    def _on_fk_response(self, fut) -> None:
        res = fut.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS or not res.pose_stamped:
            self.log_signal.emit('WARN: MoveIt /compute_fk failed.')
            return
        p = res.pose_stamped[0].pose
        xyz = (p.position.x, p.position.y, p.position.z)
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        rpy = matrix_to_rpy(quat_to_matrix(*quat))
        self.fk_verified.emit(xyz, rpy, quat, 'MoveIt /compute_fk')

    def solve_ik_async(
        self,
        xyz: Sequence[float],
        rpy: Sequence[float],
        seed: Sequence[float],
        position_only: bool,
        solver_mode: str,
    ) -> None:
        use_moveit = (
            solver_mode in ('auto', 'moveit')
            and not position_only
            and self._ik_client is not None
            and self._ik_client.service_is_ready()
        )
        if use_moveit:
            qx, qy, qz, qw = matrix_to_quat(rpy_to_matrix(*rpy))
            req = GetPositionIK.Request()
            ik_req = PositionIKRequest()
            ik_req.group_name = 'arm'
            ik_req.ik_link_name = 'ee_link'
            ik_req.avoid_collisions = True
            ik_req.timeout = RosDuration(sec=0, nanosec=250_000_000)
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose.position.x = float(xyz[0])
            ps.pose.position.y = float(xyz[1])
            ps.pose.position.z = float(xyz[2])
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            ik_req.pose_stamped = ps
            ik_req.robot_state = RobotState(
                joint_state=JointState(name=list(ARM_JOINTS), position=[float(s) for s in seed])
            )
            req.ik_request = ik_req
            fut = self._ik_client.call_async(req)
            fut.add_done_callback(
                lambda f: self._on_moveit_ik_done(f, xyz, rpy, seed, position_only, solver_mode)
            )
            return

        if solver_mode == 'moveit':
            if position_only:
                self.ik_solved.emit(False, list(seed), 'MoveIt /compute_ik requires 6-DOF pose; uncheck Position-Only or use Auto.')
            else:
                self.ik_solved.emit(False, list(seed), 'MoveIt /compute_ik service unavailable.')
            return

        ok, q_sol, pos_err, rot_err = compute_ik_local(
            xyz, target_rpy=None if position_only else rpy, seed_joints=seed, frame='world', position_only=position_only
        )
        detail = f'Local DLS IK: pos_err={pos_err * 1000:.2f} mm, rot_err={math.degrees(rot_err):.2f} deg'
        self.ik_solved.emit(ok, [float(v) for v in q_sol], detail)

    def _on_moveit_ik_done(self, fut, xyz, rpy, seed, position_only, solver_mode) -> None:
        res = fut.result()
        if res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS:
            name_map = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            sol = [float(name_map[j]) for j in ARM_JOINTS]
            self.ik_solved.emit(True, sol, 'MoveIt /compute_ik converged (collision-checked)')
            return

        if solver_mode == 'auto':
            ok, q_sol, pos_err, rot_err = compute_ik_local(
                xyz, target_rpy=rpy, seed_joints=seed, frame='world', position_only=position_only
            )
            err_val = res.error_code.val if res else 'timeout'
            detail = (
                f'MoveIt IK err={err_val}; Local DLS fallback: '
                f'pos_err={pos_err * 1000:.2f} mm, rot_err={math.degrees(rot_err):.2f} deg'
            )
            self.ik_solved.emit(ok, [float(v) for v in q_sol], detail)
        else:
            err_val = res.error_code.val if res else 'timeout'
            self.ik_solved.emit(False, list(seed), f'MoveIt /compute_ik failed (error_code={err_val})')

    def moveit_plan_and_execute_joints(
        self, joints: Sequence[float], vel_scale: float, pipeline: str = 'ompl', planner_id: str = ''
    ) -> None:
        if self._moveit_client is None or not self._moveit_client.server_is_ready():
            self.log_signal.emit('WARN: MoveIt /move_action server unavailable.')
            self.motion_finished.emit(False, 'MoveIt /move_action unavailable')
            return

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.pipeline_id = pipeline
        if planner_id:
            req.planner_id = planner_id
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = max(0.05, min(1.0, vel_scale))
        req.max_acceleration_scaling_factor = max(0.05, min(1.0, vel_scale))

        c = Constraints()
        for jname, jval in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jval)
            jc.tolerance_above = 0.002
            jc.tolerance_below = 0.002
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]

        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)

        self.log_signal.emit(f'MoveIt ({pipeline}) planning & executing joint goal...')
        fut = self._moveit_client.send_goal_async(goal)
        fut.add_done_callback(self._on_moveit_goal_response)

    def moveit_plan_and_execute_pose(
        self,
        xyz: Sequence[float],
        rpy: Sequence[float],
        vel_scale: float,
        position_only: bool,
        pipeline: str = 'ompl',
        planner_id: str = '',
    ) -> None:
        if self._moveit_client is None or not self._moveit_client.server_is_ready():
            self.log_signal.emit('WARN: MoveIt /move_action server unavailable.')
            self.motion_finished.emit(False, 'MoveIt /move_action unavailable')
            return

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.pipeline_id = pipeline
        if planner_id:
            req.planner_id = planner_id
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = max(0.05, min(1.0, vel_scale))
        req.max_acceleration_scaling_factor = max(0.05, min(1.0, vel_scale))

        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = 'ee_link'
        pc.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.004])
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0
        pc.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[pose])
        c.position_constraints.append(pc)

        if not position_only:
            qx, qy, qz, qw = matrix_to_quat(rpy_to_matrix(*rpy))
            oc = OrientationConstraint()
            oc.header.frame_id = 'world'
            oc.link_name = 'ee_link'
            oc.orientation.x = qx
            oc.orientation.y = qy
            oc.orientation.z = qz
            oc.orientation.w = qw
            oc.absolute_x_axis_tolerance = 0.02
            oc.absolute_y_axis_tolerance = 0.02
            oc.absolute_z_axis_tolerance = 0.02
            oc.weight = 1.0
            c.orientation_constraints.append(oc)

        req.goal_constraints = [c]
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)

        self.log_signal.emit(f'MoveIt ({pipeline}) planning & executing pose goal...')
        fut = self._moveit_client.send_goal_async(goal)
        fut.add_done_callback(self._on_moveit_goal_response)

    def _on_moveit_goal_response(self, fut) -> None:
        gh = fut.result()
        if not gh or not gh.accepted:
            self.log_signal.emit('ERROR: MoveIt goal rejected.')
            self.motion_finished.emit(False, 'MoveIt goal rejected')
            return
        self._active_moveit_handle = gh
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_moveit_result)

    def _on_moveit_result(self, fut) -> None:
        self._active_moveit_handle = None
        res = fut.result().result
        ok = res.error_code.val == MoveItErrorCodes.SUCCESS
        msg = 'MoveIt plan & execute succeeded' if ok else f'MoveIt failed (error_code={res.error_code.val})'
        self.log_signal.emit(msg)
        self.motion_finished.emit(ok, msg)

    def emergency_stop(self) -> None:
        if self._active_traj_handle is not None:
            self._active_traj_handle.cancel_goal_async()
        if self._active_moveit_handle is not None:
            self._active_moveit_handle.cancel_goal_async()
        # Hold current position immediately via direct topic + trajectory fallback
        self.stream_joint_positions_rt(self._latest_joints, reach_time_sec=0.05)
        self.send_joint_trajectory(self._latest_joints, 0.20)
        self.log_signal.emit('E-STOP TRIGGERED: Canceled active goals and commanded hold at actual joints.')


class MainWindow(QtWidgets.QMainWindow):
    """Industrial Teach Pendant & Control Studio Main Window."""

    def __init__(self, ros_args: Optional[List[str]] = None, enable_ros: bool = True) -> None:
        super().__init__()
        self.setWindowTitle('SML Arm Studio — MoveIt 2 & ros2_control Teach Pendant')
        self.resize(1460, 860)

        self._use_degrees = True
        self._actual_joints = list(POSES['home'])
        self._target_joints = list(POSES['home'])
        self._actual_gripper = 0.038
        self._target_gripper = 0.038
        self._updating_widgets = False
        self._execute_next_ik = False
        self._sequence_running = False
        self._sequence_idx = 0
        self._first_js_received = False
        self._rt_pending = False
        self._rt_in_cooldown = False
        self._ui_dirty = False
        self._latest_js_hz = 0.0

        self.bridge = RosBridgeWorker(ros_args=ros_args)
        self.bridge.joint_states_received.connect(self._on_joint_states)
        self.bridge.gripper_state_received.connect(self._on_gripper_state)
        self.bridge.connection_status.connect(self._on_connection_status)
        self.bridge.log_signal.connect(self._log)
        self.bridge.fk_verified.connect(self._on_fk_verified)
        self.bridge.ik_solved.connect(self._on_ik_solved)
        self.bridge.motion_finished.connect(self._on_motion_finished)

        self._rt_cooldown_timer = QtCore.QTimer(self)
        self._rt_cooldown_timer.setSingleShot(True)
        self._rt_cooldown_timer.setInterval(33)
        self._rt_cooldown_timer.timeout.connect(self._on_rt_cooldown_done)

        self._ui_refresh_timer = QtCore.QTimer(self)
        self._ui_refresh_timer.setInterval(50)
        self._ui_refresh_timer.timeout.connect(self._flush_ui_refresh)
        self._ui_refresh_timer.start()

        self._build_ui()
        self._sync_target_widgets()
        self._refresh_fk_labels()

        if enable_ros:
            self.bridge.start()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root_layout = QtWidgets.QVBoxLayout(central)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(6)

        # --- Top Status & Safety Bar ---
        top_bar = QtWidgets.QHBoxLayout()
        title_lbl = QtWidgets.QLabel('SML-ARM CONTROL STUDIO')
        title_lbl.setStyleSheet('font-size: 15px; font-weight: bold; color: #38bdf8; letter-spacing: 1px;')
        top_bar.addWidget(title_lbl)

        self.badge_js = self._make_badge('JointStates: OFFLINE')
        self.badge_ctrl = self._make_badge('ros2_control: OFFLINE')
        self.badge_moveit = self._make_badge('MoveIt 2: OFFLINE')
        for b in (self.badge_js, self.badge_ctrl, self.badge_moveit):
            top_bar.addWidget(b)

        top_bar.addStretch()

        top_bar.addWidget(QtWidgets.QLabel('Duration (s):'))
        self.spin_duration = QtWidgets.QDoubleSpinBox()
        self.spin_duration.setRange(0.2, 20.0)
        self.spin_duration.setSingleStep(0.25)
        self.spin_duration.setValue(2.0)
        top_bar.addWidget(self.spin_duration)

        top_bar.addWidget(QtWidgets.QLabel('Vel %:'))
        self.spin_vel = QtWidgets.QSpinBox()
        self.spin_vel.setRange(5, 100)
        self.spin_vel.setSingleStep(5)
        self.spin_vel.setValue(50)
        top_bar.addWidget(self.spin_vel)

        self.chk_live_jog = QtWidgets.QCheckBox('Real-Time Jog (Instant)')
        self.chk_live_jog.setChecked(True)
        self.chk_live_jog.setToolTip(
            'Stream joint and Cartesian jog commands immediately via /arm_trajectory_controller/joint_trajectory.'
        )
        top_bar.addWidget(self.chk_live_jog)

        self.btn_estop = QtWidgets.QPushButton('EMERGENCY STOP')
        self.btn_estop.setObjectName('btnEstop')
        self.btn_estop.clicked.connect(self._on_estop)
        top_bar.addWidget(self.btn_estop)
        root_layout.addLayout(top_bar)

        # --- Main 3-Column Splitter ---
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self._build_joint_panel())
        splitter.addWidget(self._build_kinematics_panel())
        splitter.addWidget(self._build_viewport_and_sequencer_panel())
        splitter.setSizes([430, 460, 550])
        root_layout.addWidget(splitter, stretch=1)

        # --- Bottom Console ---
        self.log_console = QtWidgets.QPlainTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumHeight(115)
        root_layout.addWidget(self.log_console)

    def _make_badge(self, text: str) -> QtWidgets.QLabel:
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet(
            'background-color: #291d21; color: #fca5a5; border: 1px solid #7f1d1d; '
            'border-radius: 4px; padding: 3px 8px; font-weight: bold; font-size: 11px;'
        )
        return lbl

    def _set_badge_state(self, lbl: QtWidgets.QLabel, text: str, ok: bool) -> None:
        if ok:
            lbl.setStyleSheet(
                'background-color: #122b24; color: #6ee7b7; border: 1px solid #047857; '
                'border-radius: 4px; padding: 3px 8px; font-weight: bold; font-size: 11px;'
            )
        else:
            lbl.setStyleSheet(
                'background-color: #291d21; color: #fca5a5; border: 1px solid #7f1d1d; '
                'border-radius: 4px; padding: 3px 8px; font-weight: bold; font-size: 11px;'
            )
        lbl.setText(text)

    def _build_joint_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)

        grp = QtWidgets.QGroupBox('1. Joint Teach Pendant (6-DOF)')
        v = QtWidgets.QVBoxLayout(grp)

        # Unit + Jog Step toolbar
        bar = QtWidgets.QHBoxLayout()
        self.btn_unit = QtWidgets.QPushButton('Units: Degrees (°)')
        self.btn_unit.setCheckable(True)
        self.btn_unit.clicked.connect(self._toggle_units)
        bar.addWidget(self.btn_unit)

        bar.addWidget(QtWidgets.QLabel('Jog Step:'))
        self.cmb_joint_step = QtWidgets.QComboBox()
        self.cmb_joint_step.addItems(['0.5°', '1°', '5°', '15°'])
        self.cmb_joint_step.setCurrentIndex(2)
        bar.addWidget(self.cmb_joint_step)
        v.addLayout(bar)

        self.joint_actual_lbls: List[QtWidgets.QLabel] = []
        self.joint_sliders: List[QtWidgets.QSlider] = []
        self.joint_spins: List[QtWidgets.QDoubleSpinBox] = []

        for idx, (jname, (lo, hi)) in enumerate(zip(ARM_JOINTS, JOINT_LIMITS)):
            row_box = QtWidgets.QFrame()
            row_box.setStyleSheet('QFrame { background-color: #131a22; border-radius: 4px; padding: 3px; }')
            rl = QtWidgets.QVBoxLayout(row_box)
            rl.setContentsMargins(6, 4, 6, 4)
            rl.setSpacing(3)

            hdr = QtWidgets.QHBoxLayout()
            name_lbl = QtWidgets.QLabel(f'J{idx + 1} ({jname})')
            name_lbl.setStyleSheet('font-weight: bold; color: #e2e8f0;')
            act_lbl = QtWidgets.QLabel('Actual: 0.00°')
            act_lbl.setStyleSheet('color: #38bdf8; font-family: monospace;')
            lim_lbl = QtWidgets.QLabel(f'[{math.degrees(lo):.0f}° .. {math.degrees(hi):.0f}°]')
            lim_lbl.setStyleSheet('color: #64748b; font-size: 10px;')
            hdr.addWidget(name_lbl)
            hdr.addStretch()
            hdr.addWidget(act_lbl)
            hdr.addWidget(lim_lbl)
            rl.addLayout(hdr)

            ctrl = QtWidgets.QHBoxLayout()
            btn_minus = QtWidgets.QPushButton('-')
            btn_minus.setFixedWidth(30)
            btn_minus.setAutoRepeat(True)
            btn_minus.setAutoRepeatDelay(150)
            btn_minus.setAutoRepeatInterval(40)
            btn_minus.clicked.connect(lambda _c, i=idx: self._jog_joint(i, -1))

            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-1800, 1800)
            slider.valueChanged.connect(lambda val, i=idx: self._on_slider_changed(i, val))

            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.setFixedWidth(82)
            spin.valueChanged.connect(lambda val, i=idx: self._on_spin_changed(i, val))

            btn_plus = QtWidgets.QPushButton('+')
            btn_plus.setFixedWidth(30)
            btn_plus.setAutoRepeat(True)
            btn_plus.setAutoRepeatDelay(150)
            btn_plus.setAutoRepeatInterval(40)
            btn_plus.clicked.connect(lambda _c, i=idx: self._jog_joint(i, +1))

            ctrl.addWidget(btn_minus)
            ctrl.addWidget(slider, stretch=1)
            ctrl.addWidget(spin)
            ctrl.addWidget(btn_plus)
            rl.addLayout(ctrl)

            v.addWidget(row_box)
            self.joint_actual_lbls.append(act_lbl)
            self.joint_sliders.append(slider)
            self.joint_spins.append(spin)

        # Execution buttons
        act_grid = QtWidgets.QGridLayout()
        btn_exec = QtWidgets.QPushButton('Execute Target (ros2_control)')
        btn_exec.setObjectName('btnPrimary')
        btn_exec.clicked.connect(self._execute_target_joints)

        btn_moveit_exec = QtWidgets.QPushButton('Plan & Execute (MoveIt 2)')
        btn_moveit_exec.setObjectName('btnMoveIt')
        btn_moveit_exec.clicked.connect(self._moveit_execute_target_joints)

        btn_sync = QtWidgets.QPushButton('Sync Target <- Actual')
        btn_sync.clicked.connect(self._sync_from_actual)

        btn_zero = QtWidgets.QPushButton('Zero Target (Home)')
        btn_zero.clicked.connect(lambda: self._apply_preset('home', execute=False))

        act_grid.addWidget(btn_exec, 0, 0)
        act_grid.addWidget(btn_moveit_exec, 0, 1)
        act_grid.addWidget(btn_sync, 1, 0)
        act_grid.addWidget(btn_zero, 1, 1)
        v.addLayout(act_grid)

        # Gripper End-Effector Control Box
        grip_box = QtWidgets.QFrame()
        grip_box.setStyleSheet('QFrame { background-color: #131a22; border-radius: 4px; padding: 4px; }')
        gl = QtWidgets.QVBoxLayout(grip_box)
        gl.setContentsMargins(6, 4, 6, 4)
        gl.setSpacing(4)

        ghdr = QtWidgets.QHBoxLayout()
        gtitle = QtWidgets.QLabel('End-Effector Gripper')
        gtitle.setStyleSheet('font-weight: bold; color: #fb923c;')
        self.lbl_gripper_actual = QtWidgets.QLabel('Actual: 38.0 mm | Target: 38.0 mm')
        self.lbl_gripper_actual.setStyleSheet('color: #93c5fd; font-family: monospace; font-size: 11px;')
        ghdr.addWidget(gtitle)
        ghdr.addStretch()
        ghdr.addWidget(self.lbl_gripper_actual)
        gl.addLayout(ghdr)

        gctrl = QtWidgets.QHBoxLayout()
        self.slider_gripper = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_gripper.setRange(0, 400)
        self.slider_gripper.setValue(380)
        self.slider_gripper.valueChanged.connect(self._on_gripper_slider_changed)
        btn_g_open = QtWidgets.QPushButton('Open (38mm)')
        btn_g_open.clicked.connect(lambda: self._set_gripper_width(0.038, publish=True))
        btn_g_grasp = QtWidgets.QPushButton('Grasp (15.5mm)')
        btn_g_grasp.clicked.connect(lambda: self._set_gripper_width(0.0155, publish=True))
        btn_g_close = QtWidgets.QPushButton('Close (0mm)')
        btn_g_close.clicked.connect(lambda: self._set_gripper_width(0.0, publish=True))
        gctrl.addWidget(self.slider_gripper, stretch=1)
        gctrl.addWidget(btn_g_open)
        gctrl.addWidget(btn_g_grasp)
        gctrl.addWidget(btn_g_close)
        gl.addLayout(gctrl)
        v.addWidget(grip_box)

        layout.addWidget(grp)
        return panel

    def _build_kinematics_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)

        # --- Live FK Card ---
        fk_grp = QtWidgets.QGroupBox('2. Forward Kinematics (Live TCP Pose)')
        fk_lay = QtWidgets.QVBoxLayout(fk_grp)
        self.lbl_fk_world = QtWidgets.QLabel()
        self.lbl_fk_world.setStyleSheet('font-family: monospace; font-size: 11px; color: #e2e8f0;')
        self.lbl_fk_base = QtWidgets.QLabel()
        self.lbl_fk_base.setStyleSheet('font-family: monospace; font-size: 11px; color: #94a3b8;')
        self.lbl_fk_moveit = QtWidgets.QLabel('MoveIt /compute_fk: click Verify to query move_group')
        self.lbl_fk_moveit.setStyleSheet('font-family: monospace; font-size: 11px; color: #2dd4bf;')

        btn_verify_fk = QtWidgets.QPushButton('Verify Actual Pose via MoveIt /compute_fk')
        btn_verify_fk.clicked.connect(lambda: self.bridge.verify_moveit_fk(self._actual_joints))

        fk_lay.addWidget(self.lbl_fk_world)
        fk_lay.addWidget(self.lbl_fk_base)
        fk_lay.addWidget(self.lbl_fk_moveit)
        fk_lay.addWidget(btn_verify_fk)
        layout.addWidget(fk_grp)

        # --- Cartesian TCP Jog Pad ---
        jog_grp = QtWidgets.QGroupBox('3. Cartesian TCP Jog Pad (World Frame)')
        jog_v = QtWidgets.QVBoxLayout(jog_grp)
        step_row = QtWidgets.QHBoxLayout()
        step_row.addWidget(QtWidgets.QLabel('Linear Step:'))
        self.cmb_cart_lin = QtWidgets.QComboBox()
        self.cmb_cart_lin.addItems(['1 mm', '5 mm', '10 mm', '25 mm', '50 mm'])
        self.cmb_cart_lin.setCurrentIndex(2)
        step_row.addWidget(self.cmb_cart_lin)
        step_row.addWidget(QtWidgets.QLabel('Angular Step:'))
        self.cmb_cart_rot = QtWidgets.QComboBox()
        self.cmb_cart_rot.addItems(['1°', '5°', '15°'])
        self.cmb_cart_rot.setCurrentIndex(1)
        step_row.addWidget(self.cmb_cart_rot)
        jog_v.addLayout(step_row)

        jog_grid = QtWidgets.QGridLayout()
        axes = [('X', 0, True), ('Y', 1, True), ('Z', 2, True), ('Roll', 3, False), ('Pitch', 4, False), ('Yaw', 5, False)]
        for col_idx, (label, axis_idx, is_lin) in enumerate(axes[:3]):
            bm = QtWidgets.QPushButton(f'-{label}')
            bp = QtWidgets.QPushButton(f'+{label}')
            for btn in (bm, bp):
                btn.setAutoRepeat(True)
                btn.setAutoRepeatDelay(150)
                btn.setAutoRepeatInterval(40)
            bm.clicked.connect(lambda _c, a=axis_idx, s=-1, lin=is_lin: self._cartesian_jog(a, s, lin))
            bp.clicked.connect(lambda _c, a=axis_idx, s=+1, lin=is_lin: self._cartesian_jog(a, s, lin))
            jog_grid.addWidget(bp, 0, col_idx)
            jog_grid.addWidget(bm, 1, col_idx)
        for col_idx, (label, axis_idx, is_lin) in enumerate(axes[3:]):
            bm = QtWidgets.QPushButton(f'-{label}')
            bp = QtWidgets.QPushButton(f'+{label}')
            for btn in (bm, bp):
                btn.setAutoRepeat(True)
                btn.setAutoRepeatDelay(150)
                btn.setAutoRepeatInterval(40)
            bm.clicked.connect(lambda _c, a=axis_idx, s=-1, lin=is_lin: self._cartesian_jog(a, s, lin))
            bp.clicked.connect(lambda _c, a=axis_idx, s=+1, lin=is_lin: self._cartesian_jog(a, s, lin))
            jog_grid.addWidget(bp, 2, col_idx)
            jog_grid.addWidget(bm, 3, col_idx)
        jog_v.addLayout(jog_grid)
        layout.addWidget(jog_grp)

        # --- IK Solver & Pose Planning ---
        ik_grp = QtWidgets.QGroupBox('4. Inverse Kinematics (IK) & MoveIt Pose Planner')
        ik_v = QtWidgets.QVBoxLayout(ik_grp)

        pose_grid = QtWidgets.QGridLayout()
        self.ik_spins: List[QtWidgets.QDoubleSpinBox] = []
        pose_specs = [
            ('X (m)', -1.2, 1.2, 0.01, 3),
            ('Y (m)', -1.2, 1.2, 0.01, 3),
            ('Z (m)', 0.0, 1.3, 0.01, 3),
            ('Roll (°)', -180.0, 180.0, 5.0, 1),
            ('Pitch (°)', -180.0, 180.0, 5.0, 1),
            ('Yaw (°)', -180.0, 180.0, 5.0, 1),
        ]
        for i, (lbl, lo, hi, step, dec) in enumerate(pose_specs):
            r, c = divmod(i, 3)
            pose_grid.addWidget(QtWidgets.QLabel(lbl), r * 2, c)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setSingleStep(step)
            sp.setDecimals(dec)
            pose_grid.addWidget(sp, r * 2 + 1, c)
            self.ik_spins.append(sp)
        ik_v.addLayout(pose_grid)

        opt_row = QtWidgets.QHBoxLayout()
        self.chk_pos_only = QtWidgets.QCheckBox('Position-Only (3D XYZ)')
        opt_row.addWidget(self.chk_pos_only)
        opt_row.addWidget(QtWidgets.QLabel('IK Solver:'))
        self.cmb_ik_solver = QtWidgets.QComboBox()
        self.cmb_ik_solver.addItems(['auto', 'moveit', 'local'])
        opt_row.addWidget(self.cmb_ik_solver)
        opt_row.addWidget(QtWidgets.QLabel('Pipeline:'))
        self.cmb_pipeline = QtWidgets.QComboBox()
        self.cmb_pipeline.addItems(['ompl', 'pilz_industrial_motion_planner'])
        opt_row.addWidget(self.cmb_pipeline)
        ik_v.addLayout(opt_row)

        btn_grid = QtWidgets.QGridLayout()
        btn_copy_ee = QtWidgets.QPushButton('Copy Current EE')
        btn_copy_ee.clicked.connect(self._copy_current_ee_to_ik)
        btn_solve_ik = QtWidgets.QPushButton('Solve IK (Preview)')
        btn_solve_ik.clicked.connect(lambda: self._request_ik(execute=False))
        btn_exec_ik = QtWidgets.QPushButton('Solve & Execute IK')
        btn_exec_ik.setObjectName('btnPrimary')
        btn_exec_ik.clicked.connect(lambda: self._request_ik(execute=True))
        btn_moveit_pose = QtWidgets.QPushButton('MoveIt Plan to Pose')
        btn_moveit_pose.setObjectName('btnMoveIt')
        btn_moveit_pose.clicked.connect(self._moveit_plan_to_pose)

        btn_grid.addWidget(btn_copy_ee, 0, 0)
        btn_grid.addWidget(btn_solve_ik, 0, 1)
        btn_grid.addWidget(btn_exec_ik, 1, 0)
        btn_grid.addWidget(btn_moveit_pose, 1, 1)
        ik_v.addLayout(btn_grid)

        self.lbl_ik_status = QtWidgets.QLabel('IK Status: Ready')
        self.lbl_ik_status.setStyleSheet('color: #93c5fd; font-size: 11px;')
        ik_v.addWidget(self.lbl_ik_status)

        layout.addWidget(ik_grp)
        layout.addStretch()
        self._copy_current_ee_to_ik()
        return panel

    def _build_viewport_and_sequencer_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)

        # --- 3D Arm Viewport ---
        vp_grp = QtWidgets.QGroupBox('5. 3D Arm Preview & Trajectory Trail')
        vp_v = QtWidgets.QVBoxLayout(vp_grp)
        self.viewport = Arm3DViewport()
        vp_v.addWidget(self.viewport, stretch=1)

        vp_bar = QtWidgets.QHBoxLayout()
        btn_reset_cam = QtWidgets.QPushButton('Reset View')
        btn_reset_cam.clicked.connect(self.viewport.reset_view)
        btn_clear_trail = QtWidgets.QPushButton('Clear Trail')
        btn_clear_trail.clicked.connect(self.viewport.clear_trail)
        vp_bar.addWidget(btn_reset_cam)
        vp_bar.addWidget(btn_clear_trail)
        vp_bar.addStretch()

        for pname in ('home', 'ready', 'inspect', 'pre_pick', 'pick', 'pre_place', 'place'):
            pb = QtWidgets.QPushButton(pname.replace('_', ' ').title())
            pb.clicked.connect(lambda _c, n=pname: self._apply_preset(n, execute=True))
            vp_bar.addWidget(pb)
        vp_v.addLayout(vp_bar)
        layout.addWidget(vp_grp, stretch=3)

        # --- Waypoint Sequencer ---
        seq_grp = QtWidgets.QGroupBox('6. Teach & Playback Waypoint Sequencer')
        seq_v = QtWidgets.QVBoxLayout(seq_grp)

        self.tbl_waypoints = QtWidgets.QTableWidget(0, 9)
        self.tbl_waypoints.setHorizontalHeaderLabels(['Name', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Dur(s)', 'Grip(mm)'])
        self.tbl_waypoints.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tbl_waypoints.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        seq_v.addWidget(self.tbl_waypoints)

        seq_btns = QtWidgets.QHBoxLayout()
        b_add_act = QtWidgets.QPushButton('Teach Actual')
        b_add_act.clicked.connect(lambda: self._add_waypoint(self._actual_joints, gripper_mm=self._actual_gripper * 1000.0))
        b_add_tgt = QtWidgets.QPushButton('Teach Target')
        b_add_tgt.clicked.connect(lambda: self._add_waypoint(self._target_joints, gripper_mm=self._target_gripper * 1000.0))
        b_go = QtWidgets.QPushButton('Move Selected')
        b_go.clicked.connect(self._move_to_selected_waypoint)
        b_del = QtWidgets.QPushButton('Delete')
        b_del.clicked.connect(self._delete_selected_waypoint)
        b_pnp_demo = QtWidgets.QPushButton('Run Pick & Place Demo')
        b_pnp_demo.setObjectName('btnMoveIt')
        b_pnp_demo.clicked.connect(self._run_pick_and_place_demo)
        seq_btns.addWidget(b_add_act)
        seq_btns.addWidget(b_add_tgt)
        seq_btns.addWidget(b_go)
        seq_btns.addWidget(b_del)
        seq_btns.addWidget(b_pnp_demo)
        seq_v.addLayout(seq_btns)

        run_btns = QtWidgets.QHBoxLayout()
        self.btn_run_seq = QtWidgets.QPushButton('Run Sequence')
        self.btn_run_seq.setObjectName('btnPrimary')
        self.btn_run_seq.clicked.connect(self._start_sequence)
        b_stop_seq = QtWidgets.QPushButton('Stop')
        b_stop_seq.clicked.connect(self._stop_sequence)
        self.chk_loop_seq = QtWidgets.QCheckBox('Loop')
        b_save = QtWidgets.QPushButton('Save JSON')
        b_save.clicked.connect(self._save_waypoints_json)
        b_load = QtWidgets.QPushButton('Load JSON')
        b_load.clicked.connect(self._load_waypoints_json)
        run_btns.addWidget(self.btn_run_seq)
        run_btns.addWidget(b_stop_seq)
        run_btns.addWidget(self.chk_loop_seq)
        run_btns.addStretch()
        run_btns.addWidget(b_save)
        run_btns.addWidget(b_load)
        seq_v.addLayout(run_btns)

        layout.addWidget(seq_grp, stretch=2)
        return panel

    # --- Joint Widget Sync & Units ---
    def _toggle_units(self) -> None:
        self._use_degrees = not self.btn_unit.isChecked()
        self.btn_unit.setText('Units: Degrees (°)' if self._use_degrees else 'Units: Radians (rad)')
        self._sync_target_widgets()
        self._update_actual_labels()

    def _sync_target_widgets(self) -> None:
        self._updating_widgets = True
        try:
            for i, (lo, hi) in enumerate(JOINT_LIMITS):
                val = self._target_joints[i]
                slider = self.joint_sliders[i]
                spin = self.joint_spins[i]
                if self._use_degrees:
                    slider.setRange(int(round(math.degrees(lo) * 10)), int(round(math.degrees(hi) * 10)))
                    slider.setValue(int(round(math.degrees(val) * 10)))
                    spin.setRange(math.degrees(lo), math.degrees(hi))
                    spin.setDecimals(2)
                    spin.setSingleStep(1.0)
                    spin.setSuffix('°')
                    spin.setValue(math.degrees(val))
                else:
                    slider.setRange(int(round(lo * 1000)), int(round(hi * 1000)))
                    slider.setValue(int(round(val * 1000)))
                    spin.setRange(lo, hi)
                    spin.setDecimals(3)
                    spin.setSingleStep(0.05)
                    spin.setSuffix(' rad')
                    spin.setValue(val)
        finally:
            self._updating_widgets = False
        self.viewport.set_target_joints(self._target_joints)
        self._refresh_fk_labels()

    def _rt_reach_time(self) -> float:
        max_delta = max(
            abs(t - a) for t, a in zip(self._target_joints, self._actual_joints)
        )
        # Keep angular velocity strictly within URDF limits even on fast slider flicks
        max_vel = max(0.25, 2.0 * (self.spin_vel.value() / 100.0))
        base_dt = max(0.04, 0.08 * (50.0 / max(10, self.spin_vel.value())))
        return max(base_dt, max_delta / max_vel)

    def _queue_rt_stream(self) -> None:
        if not self._rt_in_cooldown:
            self._rt_pending = False
            self._rt_in_cooldown = True
            self.bridge.stream_joint_positions_rt(self._target_joints, self._rt_reach_time())
            self._rt_cooldown_timer.start()
        else:
            self._rt_pending = True

    def _on_rt_cooldown_done(self) -> None:
        if self._rt_pending:
            self._rt_pending = False
            self.bridge.stream_joint_positions_rt(self._target_joints, self._rt_reach_time())
            self._rt_cooldown_timer.start()
        else:
            self._rt_in_cooldown = False

    def _on_slider_changed(self, idx: int, raw: int) -> None:
        if self._updating_widgets:
            return
        self._target_joints[idx] = math.radians(raw / 10.0) if self._use_degrees else (raw / 1000.0)
        self._updating_widgets = True
        self.joint_spins[idx].setValue(math.degrees(self._target_joints[idx]) if self._use_degrees else self._target_joints[idx])
        self._updating_widgets = False
        self.viewport.set_target_joints(self._target_joints)
        self._refresh_fk_labels()
        if self.chk_live_jog.isChecked():
            self._queue_rt_stream()

    def _on_spin_changed(self, idx: int, val: float) -> None:
        if self._updating_widgets:
            return
        self._target_joints[idx] = math.radians(val) if self._use_degrees else val
        self._updating_widgets = True
        self.joint_sliders[idx].setValue(int(round(val * 10)) if self._use_degrees else int(round(val * 1000)))
        self._updating_widgets = False
        self.viewport.set_target_joints(self._target_joints)
        self._refresh_fk_labels()
        if self.chk_live_jog.isChecked():
            self._queue_rt_stream()

    def _jog_joint(self, idx: int, sign: int) -> None:
        step_deg = float(self.cmb_joint_step.currentText().rstrip('°'))
        self._target_joints[idx] += sign * math.radians(step_deg)
        self._target_joints = list(clamp_joints(self._target_joints))
        self._sync_target_widgets()
        if self.chk_live_jog.isChecked():
            self._queue_rt_stream()

    def _update_actual_labels(self) -> None:
        for i, val in enumerate(self._actual_joints):
            if self._use_degrees:
                self.joint_actual_lbls[i].setText(f'Actual: {math.degrees(val):+.2f}°')
            else:
                self.joint_actual_lbls[i].setText(f'Actual: {val:+.3f} rad')

    # --- FK & IK Handlers ---
    def _refresh_fk_labels(self) -> None:
        fk_w = compute_fk_local(self._actual_joints, frame='world')
        fk_b = compute_fk_local(self._actual_joints, frame='base_link')
        xw, yw, zw = fk_w['xyz']
        rw, pw, yaw_w = [math.degrees(a) for a in fk_w['rpy']]
        xb, yb, zb = fk_b['xyz']
        self.lbl_fk_world.setText(
            f'World -> EE : XYZ=[{xw:+.4f}, {yw:+.4f}, {zw:+.4f}] m  |  '
            f'RPY=[{rw:+.1f}°, {pw:+.1f}°, {yaw_w:+.1f}°]'
        )
        self.lbl_fk_base.setText(f'Base  -> EE : XYZ=[{xb:+.4f}, {yb:+.4f}, {zb:+.4f}] m')

    def _on_fk_verified(self, xyz, rpy, quat, source: str) -> None:
        del quat
        r, p, y = [math.degrees(a) for a in rpy]
        self.lbl_fk_moveit.setText(
            f'{source}: XYZ=[{xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f}] m  RPY=[{r:+.1f}°, {p:+.1f}°, {y:+.1f}°]'
        )

    def _copy_current_ee_to_ik(self) -> None:
        fk = compute_fk_local(self._actual_joints, frame='world')
        vals = list(fk['xyz']) + [math.degrees(a) for a in fk['rpy']]
        for sp, v in zip(self.ik_spins, vals):
            sp.setValue(v)

    def _cartesian_jog(self, axis_idx: int, sign: int, is_linear: bool) -> None:
        # Jog relative to target pose using fast warm-started DLS IK
        fk = compute_fk_local(self._target_joints, frame='world')
        xyz = list(fk['xyz'])
        rpy = list(fk['rpy'])
        if is_linear:
            step_mm = float(self.cmb_cart_lin.currentText().split()[0])
            xyz[axis_idx] += sign * (step_mm / 1000.0)
        else:
            step_deg = float(self.cmb_cart_rot.currentText().rstrip('°'))
            rpy[axis_idx - 3] += sign * math.radians(step_deg)

        ok, q_sol, pos_err, rot_err = compute_ik_local(
            xyz,
            target_rpy=rpy,
            seed_joints=self._target_joints,
            frame='world',
            position_only=False,
            fast_jog=True,
        )
        if not ok:
            self._log(
                f'WARN: Cartesian jog unreachable (pos_err={pos_err * 1000:.1f} mm, '
                f'rot_err={math.degrees(rot_err):.1f}°).'
            )
            return
        self._target_joints = [float(v) for v in q_sol]
        self._sync_target_widgets()
        for sp, v in zip(self.ik_spins, xyz + [math.degrees(a) for a in rpy]):
            sp.blockSignals(True)
            sp.setValue(v)
            sp.blockSignals(False)
        if self.chk_live_jog.isChecked():
            self._queue_rt_stream()

    def _request_ik(self, execute: bool) -> None:
        xyz = [self.ik_spins[0].value(), self.ik_spins[1].value(), self.ik_spins[2].value()]
        rpy = [math.radians(self.ik_spins[i].value()) for i in (3, 4, 5)]
        self._execute_next_ik = execute
        self.lbl_ik_status.setText('IK Status: Solving...')
        self.bridge.solve_ik_async(
            xyz,
            rpy,
            self._target_joints,
            self.chk_pos_only.isChecked(),
            self.cmb_ik_solver.currentText(),
        )

    def _on_ik_solved(self, ok: bool, joints: list, detail: str) -> None:
        self.lbl_ik_status.setText(f'IK Status: {detail}')
        self._log(detail)
        if ok:
            self._target_joints = list(clamp_joints(joints))
            self._sync_target_widgets()
            if self._execute_next_ik:
                self._execute_next_ik = False
                self._execute_target_joints()
        else:
            self._execute_next_ik = False

    # --- Execution & Presets ---
    def _execute_target_joints(self) -> None:
        self.bridge.send_joint_trajectory(self._target_joints, self.spin_duration.value())

    def _moveit_execute_target_joints(self) -> None:
        pipeline = self.cmb_pipeline.currentText()
        planner_id = 'PTP' if pipeline == 'pilz_industrial_motion_planner' else ''
        self.bridge.moveit_plan_and_execute_joints(
            self._target_joints, self.spin_vel.value() / 100.0, pipeline=pipeline, planner_id=planner_id
        )

    def _moveit_plan_to_pose(self) -> None:
        xyz = [self.ik_spins[0].value(), self.ik_spins[1].value(), self.ik_spins[2].value()]
        rpy = [math.radians(self.ik_spins[i].value()) for i in (3, 4, 5)]
        pipeline = self.cmb_pipeline.currentText()
        planner_id = 'PTP' if pipeline == 'pilz_industrial_motion_planner' else ''
        self.bridge.moveit_plan_and_execute_pose(
            xyz,
            rpy,
            self.spin_vel.value() / 100.0,
            self.chk_pos_only.isChecked(),
            pipeline=pipeline,
            planner_id=planner_id,
        )

    def _apply_preset(self, name: str, execute: bool) -> None:
        self._target_joints = list(POSES[name])
        self._sync_target_widgets()
        self._log(f'Preset "{name}" loaded.')
        if execute:
            self._execute_target_joints()

    def _sync_from_actual(self) -> None:
        self._target_joints = list(self._actual_joints)
        self._sync_target_widgets()
        self._log('Synced target joints to actual robot state.')

    def _on_estop(self) -> None:
        self._stop_sequence()
        self.bridge.emergency_stop()

    # --- Gripper Controls ---
    def _on_gripper_slider_changed(self, raw: int) -> None:
        if self._updating_widgets:
            return
        width_m = raw / 10000.0
        self._target_gripper = width_m
        self.viewport.set_gripper_width(target_m=width_m)
        self._refresh_gripper_label()
        self.bridge.send_gripper(width_m)

    def _set_gripper_width(self, width_m: float, publish: bool = True) -> None:
        w = max(0.0, min(0.040, float(width_m)))
        self._target_gripper = w
        self._updating_widgets = True
        self.slider_gripper.setValue(int(round(w * 10000.0)))
        self._updating_widgets = False
        self.viewport.set_gripper_width(target_m=w)
        self._refresh_gripper_label()
        if publish:
            self.bridge.send_gripper(w)

    def _refresh_gripper_label(self) -> None:
        self.lbl_gripper_actual.setText(
            f'Actual: {self._actual_gripper * 1000.0:.1f} mm | Target: {self._target_gripper * 1000.0:.1f} mm'
        )

    def _on_gripper_state(self, width_m: float) -> None:
        self._actual_gripper = width_m
        self._ui_dirty = True

    # --- Waypoint Sequencer ---
    def _add_waypoint(
        self,
        joints: Sequence[float],
        name: Optional[str] = None,
        duration: Optional[float] = None,
        gripper_mm: Optional[float] = None,
    ) -> None:
        row = self.tbl_waypoints.rowCount()
        self.tbl_waypoints.insertRow(row)
        wp_name = name or f'WP_{row + 1}'
        dur = duration if duration is not None else self.spin_duration.value()
        grip_str = '-' if gripper_mm is None else f'{gripper_mm:.1f}'
        self.tbl_waypoints.setItem(row, 0, QtWidgets.QTableWidgetItem(wp_name))
        for j in range(6):
            self.tbl_waypoints.setItem(row, j + 1, QtWidgets.QTableWidgetItem(f'{joints[j]:.4f}'))
        self.tbl_waypoints.setItem(row, 7, QtWidgets.QTableWidgetItem(f'{dur:.2f}'))
        self.tbl_waypoints.setItem(row, 8, QtWidgets.QTableWidgetItem(grip_str))
        self._log(f'Added waypoint {wp_name}: {[round(v, 3) for v in joints]} grip={grip_str}mm')

    def _waypoint_from_row(self, row: int) -> Tuple[str, List[float], float, Optional[float]]:
        name = self.tbl_waypoints.item(row, 0).text()
        joints = [float(self.tbl_waypoints.item(row, j + 1).text()) for j in range(6)]
        dur = float(self.tbl_waypoints.item(row, 7).text())
        grip_item = self.tbl_waypoints.item(row, 8)
        grip_mm = None
        if grip_item is not None and grip_item.text().strip() not in ('', '-'):
            grip_mm = float(grip_item.text())
        return name, joints, dur, grip_mm

    def _move_to_selected_waypoint(self) -> None:
        row = self.tbl_waypoints.currentRow()
        if row < 0:
            self._log('Select a waypoint row first.')
            return
        name, joints, dur, grip_mm = self._waypoint_from_row(row)
        self._target_joints = joints
        self._sync_target_widgets()
        if grip_mm is not None:
            self._set_gripper_width(grip_mm / 1000.0, publish=True)
        self.bridge.send_joint_trajectory(joints, dur)
        self._log(f'Moving to waypoint {name}.')

    def _delete_selected_waypoint(self) -> None:
        row = self.tbl_waypoints.currentRow()
        if row >= 0:
            self.tbl_waypoints.removeRow(row)

    def _populate_pick_and_place_waypoints(self) -> None:
        self.tbl_waypoints.setRowCount(0)
        pnp_steps = [
            ('1_ReadyOpen', POSES['ready'], 1.2, 38.0),
            ('2_PrePick', POSES['pre_pick'], 1.5, 38.0),
            ('3_DescendPick', POSES['pick'], 1.2, 38.0),
            ('4_GraspCube', POSES['pick'], 0.6, 15.5),
            ('5_LiftCube', POSES['pre_pick'], 1.2, 15.5),
            ('6_PrePlace', POSES['pre_place'], 1.8, 15.5),
            ('7_DescendPlace', POSES['place'], 1.2, 15.5),
            ('8_ReleaseCube', POSES['place'], 0.5, 38.0),
            ('9_RetractPlace', POSES['pre_place'], 1.0, 38.0),
            ('10_ReturnReady', POSES['ready'], 1.2, 38.0),
        ]
        for wp_name, joints, dur, grip_mm in pnp_steps:
            self._add_waypoint(joints, name=wp_name, duration=dur, gripper_mm=grip_mm)

    def _run_pick_and_place_demo(self) -> None:
        self._stop_sequence()
        self._populate_pick_and_place_waypoints()
        self._log('Loaded 10-step Pick & Place demo sequence. Starting playback...')
        self._start_sequence()

    def _start_sequence(self) -> None:
        if self.tbl_waypoints.rowCount() == 0:
            self._log('Waypoint table is empty.')
            return
        self._sequence_running = True
        self._sequence_idx = 0
        self.btn_run_seq.setEnabled(False)
        self._dispatch_sequence_step()

    def _stop_sequence(self) -> None:
        if self._sequence_running:
            self._sequence_running = False
            self.btn_run_seq.setEnabled(True)
            self._log('Waypoint sequence stopped.')

    def _dispatch_sequence_step(self) -> None:
        if not self._sequence_running:
            return
        if self._sequence_idx >= self.tbl_waypoints.rowCount():
            if self.chk_loop_seq.isChecked() and self.tbl_waypoints.rowCount() > 0:
                self._sequence_idx = 0
            else:
                self._stop_sequence()
                self._log('Waypoint sequence completed.')
                return
        self.tbl_waypoints.selectRow(self._sequence_idx)
        name, joints, dur, grip_mm = self._waypoint_from_row(self._sequence_idx)
        prev_joints = list(self._target_joints)
        self._target_joints = joints
        self._sync_target_widgets()
        if grip_mm is not None:
            self._set_gripper_width(grip_mm / 1000.0, publish=True)
        self._log(f'Sequence [{self._sequence_idx + 1}/{self.tbl_waypoints.rowCount()}] -> {name}')
        max_joint_delta = max(abs(a - b) for a, b in zip(joints, prev_joints))
        max_actual_delta = max(abs(a - b) for a, b in zip(joints, self._actual_joints))
        if max_joint_delta < 1e-3 and max_actual_delta < 2e-2 and grip_mm is not None:
            # Pure gripper actuation step: wait settle duration then advance
            QtCore.QTimer.singleShot(int(max(0.2, dur) * 1000), self._advance_gripper_sequence_step)
        else:
            self.bridge.send_joint_trajectory(joints, dur)

    def _advance_gripper_sequence_step(self) -> None:
        if self._sequence_running:
            self._sequence_idx += 1
            self._dispatch_sequence_step()

    def _save_waypoints_json(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save Waypoint Program', 'sml_waypoints.json', 'JSON (*.json)')
        if not path:
            return
        data = []
        for r in range(self.tbl_waypoints.rowCount()):
            name, joints, dur, grip_mm = self._waypoint_from_row(r)
            data.append({'name': name, 'joints_rad': joints, 'duration_s': dur, 'gripper_mm': grip_mm})
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self._log(f'Saved {len(data)} waypoints to {path}')

    def _load_waypoints_json(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Load Waypoint Program', '', 'JSON (*.json)')
        if not path:
            return
        with open(path, 'r') as f:
            data = json.load(f)
        self.tbl_waypoints.setRowCount(0)
        for item in data:
            self._add_waypoint(
                item['joints_rad'],
                name=item.get('name'),
                duration=item.get('duration_s', 2.0),
                gripper_mm=item.get('gripper_mm'),
            )
        self._log(f'Loaded {len(data)} waypoints from {path}')

    # --- ROS Callbacks ---
    def _on_joint_states(self, joints: list, hz: float) -> None:
        self._actual_joints = list(joints)
        self._latest_js_hz = hz
        self._ui_dirty = True
        if not self._first_js_received:
            self._first_js_received = True
            self._target_joints = list(joints)
            self._sync_target_widgets()
            self._copy_current_ee_to_ik()

    def _flush_ui_refresh(self) -> None:
        if not self._ui_dirty:
            return
        self._ui_dirty = False
        self._update_actual_labels()
        self.viewport.set_actual_joints(self._actual_joints)
        self.viewport.set_gripper_width(actual_m=self._actual_gripper)
        self._refresh_gripper_label()
        self._refresh_fk_labels()
        self._set_badge_state(self.badge_js, f'JointStates: {self._latest_js_hz:.0f} Hz', True)

    def _on_connection_status(self, js_ok: bool, ctrl_ok: bool, moveit_ok: bool) -> None:
        if not js_ok:
            self._set_badge_state(self.badge_js, 'JointStates: OFFLINE', False)
        self._set_badge_state(self.badge_ctrl, 'ros2_control: READY' if ctrl_ok else 'ros2_control: OFFLINE', ctrl_ok)
        self._set_badge_state(self.badge_moveit, 'MoveIt 2: READY' if moveit_ok else 'MoveIt 2: OFFLINE', moveit_ok)

    def _on_motion_finished(self, ok: bool, msg: str) -> None:
        if self._sequence_running:
            if ok:
                self._sequence_idx += 1
                self._dispatch_sequence_step()
            else:
                self._stop_sequence()
                self._log(f'Sequence aborted: {msg}')

    def _log(self, text: str) -> None:
        stamp = time.strftime('%H:%M:%S')
        self.log_console.appendPlainText(f'[{stamp}] {text}')

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._rt_cooldown_timer.stop()
        self._ui_refresh_timer.stop()
        self.bridge.stop()
        super().closeEvent(event)


def run_self_test() -> int:
    """Headless verification that Qt widgets, 3D viewport, FK, and IK work cleanly."""
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)
    win = MainWindow(enable_ros=False)
    win.show()
    app.processEvents()

    # Test FK/IK integration and fast Cartesian jog inside GUI
    win._apply_preset('ready', execute=False)
    fk = compute_fk_local(POSES['ready'], frame='world')
    ok, q_sol, pos_err, _ = compute_ik_local(
        fk['xyz'], target_rpy=fk['rpy'], seed_joints=POSES['home'], frame='world'
    )
    if not ok or pos_err > 1e-3:
        print(f'SELF-TEST FAILED: IK pos_err={pos_err}', file=sys.stderr)
        return 1
    win._cartesian_jog(0, +1, True)
    win._set_gripper_width(0.0155, publish=False)
    win._populate_pick_and_place_waypoints()
    pixmap = win.viewport.grab()
    if pixmap.isNull():
        print('SELF-TEST FAILED: 3D viewport failed to render', file=sys.stderr)
        return 1
    win.close()
    print(f'GUI self-test passed (waypoints={win.tbl_waypoints.rowCount()}, ik_pos_err={pos_err * 1000:.3f} mm, live_jog={win.chk_live_jog.isChecked()}, viewport={pixmap.width()}x{pixmap.height()})')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description='SML Arm PyQt5 Control Studio & Teach Pendant.')
    parser.add_argument(
        '--self-test',
        action='store_true',
        help='Run headless Qt widget & kinematics self-test and exit.',
    )
    args, ros_args = parser.parse_known_args()

    if args.self_test:
        return run_self_test()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)
    win = MainWindow(ros_args=ros_args, enable_ros=True)
    win.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
