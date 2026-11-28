#!/usr/bin/env python3
"""Forward & Inverse Kinematics library and CLI (local analytical chain + MoveIt 2 services)."""

import argparse
import math
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS: List[str] = [
    'dof_joint1',
    'dof_joint2',
    'dof_joint3',
    'dof_joint4',
    'dof_joint5',
    'dof_joint6',
]

JOINT_LIMITS: List[Tuple[float, float]] = [
    (-3.14159, 3.14159),
    (-1.5708, 1.5708),
    (-2.35619, 2.35619),
    (-3.14159, 3.14159),
    (-2.0944, 2.0944),
    (-3.14159, 3.14159),
]

# Calibrated from firmware/mcu_controller/tools/sml_arm_gui/calibration_profile.json (rad/s, rad/s^2)
JOINT_MAX_VELOCITIES: List[float] = [
    math.radians(30.0),  # 0.5236 rad/s
    math.radians(9.0),   # 0.1571 rad/s
    math.radians(35.0),  # 0.6109 rad/s
    math.radians(40.0),  # 0.6981 rad/s
    math.radians(9.0),   # 0.1571 rad/s
    math.radians(9.0),   # 0.1571 rad/s
]

JOINT_MAX_ACCELS: List[float] = [
    math.radians(17.75), # 0.3098 rad/s^2
    math.radians(6.75),  # 0.1178 rad/s^2
    math.radians(20.75), # 0.3622 rad/s^2
    math.radians(25.75), # 0.4494 rad/s^2
    math.radians(6.75),  # 0.1178 rad/s^2
    math.radians(6.75),  # 0.1178 rad/s^2
]


def compute_segment_duration(
    q_start: Sequence[float],
    q_end: Sequence[float],
    speed_scale: float = 1.0,
    min_duration: float = 0.25,
) -> float:
    """Compute realistic trapezoidal-profile segment time (s) across all 6 joints."""
    s = max(0.05, float(speed_scale))
    worst = min_duration
    for qs, qe, vmax, amax in zip(q_start, q_end, JOINT_MAX_VELOCITIES, JOINT_MAX_ACCELS):
        dq = abs(float(qe) - float(qs))
        if dq < 1e-6:
            continue
        v = vmax * s
        a = amax * s
        t = (dq / v + v / a) if dq >= (v * v / a) else 2.0 * math.sqrt(dq / a)
        if t > worst:
            worst = t
    return worst

POSES: Dict[str, List[float]] = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, -0.45, 0.90, 0.0, 0.65, 0.0],
    'inspect': [0.75, -0.35, 0.85, 0.40, 0.70, -0.40],
    'pre_pick': [-0.5586, -0.3542, 1.1781, 0.0, -1.6097, -0.5586],
    'pick': [-0.5586, -0.4138, 1.4686, 0.0, -1.2596, -0.5586],
    'pre_place': [0.5586, -0.3542, 1.1781, 0.0, -1.6097, 0.5586],
    'place': [0.5586, -0.4138, 1.4686, 0.0, -1.2596, 0.5586],
}

# Exact kinematic chain from sml_arm_macro.xacro:
# Each entry: (child_link, xyz, rpy, revolute_axis_or_None, joint_index_or_None)
KINEMATIC_CHAIN = [
    ('base_link', (0.204085, 0.0, 0.15), (0.0, 0.0, 0.0), None, None),
    ('j1_harmonic_drive', (-0.204085, -1.38787e-17, 0.005), (0.0, 0.0, 0.0), None, None),
    ('link_1', (-3.60822e-16, 2.83012e-17, 0.038), (4.90454e-45, -7.00649e-46, -6.16298e-32), (0.0, 0.0, -1.0), 0),
    ('j2_harmonic_drive', (-0.007854, 1.51725e-07, 0.1265), (-3.23075e-30, 1.38043e-30, -5.52943e-17), None, None),
    ('link_2__1_', (0.038, -1.90582e-17, 1.13798e-15), (3.23864e-30, -7.23728e-34, 6.12323e-17), (-1.0, 0.0, 0.0), 1),
    ('j3_harmonic_drive', (0.0450535, -4.99275e-07, 0.249808), (6.93889e-18, -4.14358e-19, 7.089e-16), None, None),
    ('link_3__1_', (-0.038, -3.78671e-16, 4.10783e-15), (-3.9443e-31, -4.39112e-32, 9.24446e-32), (1.0, 0.0, 0.0), 2),
    ('j4_harmonic_drive', (-0.0372, 2.67428e-16, 0.035), (2.98372e-16, -3.67266e-17, -1.43108e-16), None, None),
    ('link_4__1_', (-1.13798e-15, -3.64086e-16, 0.066), (-6.93889e-16, -7.8889e-31, -8.40683e-20), (0.0, 2.85712e-16, -1.0), 3),
    ('j5_harmonic_drive', (-0.0378, -0.0281458, 0.20625), (-9.15909e-16, -3.58204e-16, -6.95299e-16), None, None),
    ('link_5__1_', (0.027, 0.0281458, -0.01625), (6.91342e-18, 4.93038e-31, 6.16298e-32), (-1.0, -2.77556e-16, -7.93011e-32), 4),
    ('j6_harmonic_drive', (0.0108, 3.4904e-16, 0.111), (5.20417e-16, 1.6732e-18, 4.34587e-16), None, None),
    ('ee_link', (0.0, 0.0, 0.034), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 5),
]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert ZYX Euler angles (roll, pitch, yaw) to a 3x3 rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract (roll, pitch, yaw) from a 3x3 rotation matrix."""
    sy = -R[2, 0]
    sy_clamped = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy_clamped)
    if abs(abs(sy_clamped) - 1.0) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return (roll, pitch, yaw)


def matrix_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to quaternion (qx, qy, qz, qw)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion (qx, qy, qz, qw) to 3x3 rotation matrix."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=float)


def axis_angle_to_matrix(axis: Sequence[float], theta: float) -> np.ndarray:
    """Rodrigues formula for rotation around unit axis by angle theta."""
    ax = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(ax)
    if norm < 1e-12:
        return np.eye(3)
    kx, ky, kz = ax / norm
    c = math.cos(theta)
    s = math.sin(theta)
    v = 1.0 - c
    return np.array([
        [kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
    ], dtype=float)


def _rotation_error_vector(R_target: np.ndarray, R_current: np.ndarray) -> np.ndarray:
    """Compute SO(3) axis-angle error vector taking R_current toward R_target."""
    R_err = R_target @ R_current.T
    trace = float(np.trace(R_err))
    cos_angle = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    angle = math.acos(cos_angle)
    if angle < 1e-8:
        return 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ], dtype=float)
    if abs(math.pi - angle) < 1e-4:
        eigvals, eigvecs = np.linalg.eigh(R_err)
        axis = eigvecs[:, int(np.argmax(eigvals))]
        return angle * axis
    denom = 2.0 * math.sin(angle)
    axis = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ], dtype=float) / denom
    return angle * axis


def _make_transform(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = xyz
    return T


def compute_fk_chain(
    joints: Sequence[float], frame: str = 'world'
) -> Tuple[Dict[str, np.ndarray], List[Tuple[str, np.ndarray]], List[Tuple[np.ndarray, np.ndarray]]]:
    """Compute transforms for every link, 3D chain points, and actuated joint (pos, axis) pairs."""
    if len(joints) != 6:
        raise ValueError(f'Expected 6 joint values, got {len(joints)}')

    transforms: Dict[str, np.ndarray] = {'world': np.eye(4, dtype=float)}
    chain_points: List[Tuple[str, np.ndarray]] = []
    joint_frames: List[Tuple[np.ndarray, np.ndarray]] = [None] * 6  # type: ignore[list-item]

    T_cur = np.eye(4, dtype=float)
    for idx, (link_name, xyz, rpy, axis, j_idx) in enumerate(KINEMATIC_CHAIN):
        if frame == 'base_link' and idx == 0:
            # Start at base_link identity when frame == 'base_link'
            T_cur = np.eye(4, dtype=float)
            transforms['base_link'] = T_cur.copy()
            chain_points.append(('base_link', T_cur[:3, 3].copy()))
            continue

        T_cur = T_cur @ _make_transform(xyz, rpy)
        if axis is not None and j_idx is not None:
            axis_vec = np.asarray(axis, dtype=float)
            axis_vec /= np.linalg.norm(axis_vec)
            axis_in_frame = T_cur[:3, :3] @ axis_vec
            joint_frames[j_idx] = (T_cur[:3, 3].copy(), axis_in_frame.copy())
            T_rot = np.eye(4, dtype=float)
            T_rot[:3, :3] = axis_angle_to_matrix(axis_vec, float(joints[j_idx]))
            T_cur = T_cur @ T_rot

        transforms[link_name] = T_cur.copy()
        chain_points.append((link_name, T_cur[:3, 3].copy()))

    return transforms, chain_points, joint_frames


def compute_fk_local(joints: Sequence[float], frame: str = 'world') -> Dict[str, object]:
    """Compute end-effector forward kinematics using exact URDF chain matrices."""
    transforms, chain_points, joint_frames = compute_fk_chain(joints, frame=frame)
    T_ee = transforms['ee_link']
    xyz = tuple(float(v) for v in T_ee[:3, 3])
    rpy = matrix_to_rpy(T_ee[:3, :3])
    quat = matrix_to_quat(T_ee[:3, :3])
    return {
        'frame': frame,
        'xyz': xyz,
        'rpy': rpy,
        'quat': quat,
        'matrix': T_ee,
        'transforms': transforms,
        'chain_points': chain_points,
        'joint_frames': joint_frames,
    }


def compute_jacobian(joints: Sequence[float], frame: str = 'world') -> np.ndarray:
    """Compute 6x6 geometric Jacobian [linear_vel; angular_vel] at ee_link."""
    transforms, _, joint_frames = compute_fk_chain(joints, frame=frame)
    p_ee = transforms['ee_link'][:3, 3]
    J = np.zeros((6, 6), dtype=float)
    for i in range(6):
        p_i, z_i = joint_frames[i]
        J[:3, i] = np.cross(z_i, p_ee - p_i)
        J[3:, i] = z_i
    return J


def clamp_joints(joints: Sequence[float]) -> np.ndarray:
    """Clamp 6-DOF joint vector to URDF joint limits."""
    q = np.asarray(joints, dtype=float).copy()
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        q[i] = max(lo, min(hi, q[i]))
    return q


def _solve_dls_from_seed(
    target_xyz: np.ndarray,
    R_target: Optional[np.ndarray],
    q_init: Sequence[float],
    frame: str,
    position_only: bool,
    max_iters: int,
    tol_pos: float,
    tol_rot: float,
) -> Tuple[bool, np.ndarray, float, float]:
    q = clamp_joints(q_init)
    damping = 0.03

    for _ in range(max_iters):
        fk = compute_fk_local(q, frame=frame)
        p_cur = np.asarray(fk['xyz'], dtype=float)
        R_cur = np.asarray(fk['matrix'], dtype=float)[:3, :3]

        e_pos = target_xyz - p_cur
        pos_err = float(np.linalg.norm(e_pos))
        if position_only or R_target is None:
            rot_err = 0.0
            if pos_err < tol_pos:
                return True, q, pos_err, rot_err
            J = compute_jacobian(q, frame=frame)[:3, :]
            err = e_pos
            lam = damping
            JJt = J @ J.T + (lam * lam) * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, err)
        else:
            e_rot = _rotation_error_vector(R_target, R_cur)
            rot_err = float(np.linalg.norm(e_rot))
            if pos_err < tol_pos and rot_err < tol_rot:
                return True, q, pos_err, rot_err
            J = compute_jacobian(q, frame=frame)
            err = np.concatenate([e_pos, 0.75 * e_rot])
            lam = damping
            JJt = J @ J.T + (lam * lam) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)

        max_step = float(np.max(np.abs(dq)))
        if max_step > 0.25:
            dq = dq * (0.25 / max_step)
        q = clamp_joints(q + dq)

    fk = compute_fk_local(q, frame=frame)
    pos_err = float(np.linalg.norm(target_xyz - np.asarray(fk['xyz'], dtype=float)))
    if position_only or R_target is None:
        rot_err = 0.0
        ok = pos_err < tol_pos * 5.0
    else:
        R_cur = np.asarray(fk['matrix'], dtype=float)[:3, :3]
        rot_err = float(np.linalg.norm(_rotation_error_vector(R_target, R_cur)))
        ok = (pos_err < tol_pos * 5.0) and (rot_err < tol_rot * 5.0)
    return ok, q, pos_err, rot_err


def compute_ik_local(
    target_xyz: Sequence[float],
    target_rpy: Optional[Sequence[float]] = None,
    seed_joints: Optional[Sequence[float]] = None,
    frame: str = 'world',
    position_only: bool = False,
    max_iters: int = 250,
    tol_pos: float = 1e-4,
    tol_rot: float = 1e-3,
    fast_jog: bool = False,
) -> Tuple[bool, np.ndarray, float, float]:
    """Multi-start Damped Least Squares IK solver respecting joint limits."""
    p_target = np.asarray(target_xyz, dtype=float)
    R_target = None if (position_only or target_rpy is None) else rpy_to_matrix(*target_rpy)

    if fast_jog and seed_joints is not None:
        return _solve_dls_from_seed(
            p_target,
            R_target,
            seed_joints,
            frame,
            position_only,
            max_iters=18,
            tol_pos=2e-4,
            tol_rot=2e-3,
        )

    seeds: List[List[float]] = []
    if seed_joints is not None:
        seeds.append(list(seed_joints))
    seeds.extend([
        POSES['ready'],
        POSES['home'],
        POSES['inspect'],
        POSES['pick'],
        POSES['place'],
        [0.0, 0.4, 0.9, 0.0, 0.5, 0.0],
        [1.57, -0.4, 0.9, 0.0, 0.6, 0.0],
        [-1.57, -0.4, 0.9, 0.0, 0.6, 0.0],
    ])

    best_q = clamp_joints(seeds[0])
    best_cost = float('inf')
    best_pos_err = float('inf')
    best_rot_err = float('inf')

    for seed in seeds:
        ok, q_sol, pos_err, rot_err = _solve_dls_from_seed(
            p_target, R_target, seed, frame, position_only, max_iters, tol_pos, tol_rot
        )
        cost = pos_err + 0.1 * rot_err
        if ok:
            return True, q_sol, pos_err, rot_err
        if cost < best_cost:
            best_cost = cost
            best_q = q_sol
            best_pos_err = pos_err
            best_rot_err = rot_err

    return False, best_q, best_pos_err, best_rot_err


def duration_from_seconds(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


class MoveItKinematicsClient(Node):
    """ROS 2 client for MoveIt 2 /compute_fk, /compute_ik, and trajectory execution."""

    def __init__(self) -> None:
        super().__init__('sml_arm_kinematics_cli')
        self._fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_trajectory_controller/follow_joint_trajectory',
        )

    def call_moveit_fk(
        self, joints: Sequence[float], frame_id: str = 'world', timeout_sec: float = 1.5
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float, float]]]:
        """Query MoveIt 2 /compute_fk service if available."""
        if not self._fk_client.wait_for_service(timeout_sec=timeout_sec):
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = frame_id
        req.fk_link_names = ['ee_link']
        req.robot_state = RobotState()
        req.robot_state.joint_state = JointState()
        req.robot_state.joint_state.name = list(ARM_JOINTS)
        req.robot_state.joint_state.position = [float(j) for j in joints]

        fut = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None:
            return None
        res = fut.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS or not res.pose_stamped:
            return None

        pose = res.pose_stamped[0].pose
        xyz = (pose.position.x, pose.position.y, pose.position.z)
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        rpy = matrix_to_rpy(quat_to_matrix(*quat))
        return xyz, rpy, quat

    def call_moveit_ik(
        self,
        xyz: Sequence[float],
        rpy: Sequence[float],
        seed_joints: Optional[Sequence[float]] = None,
        frame_id: str = 'world',
        avoid_collisions: bool = True,
        timeout_sec: float = 1.5,
    ) -> Tuple[Optional[List[float]], Optional[int]]:
        """Query MoveIt 2 /compute_ik service if available."""
        if not self._ik_client.wait_for_service(timeout_sec=timeout_sec):
            return None, None

        R = rpy_to_matrix(*rpy)
        qx, qy, qz, qw = matrix_to_quat(R)

        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'arm'
        ik_req.ik_link_name = 'ee_link'
        ik_req.avoid_collisions = avoid_collisions
        ik_req.timeout = duration_from_seconds(0.2)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position.x = float(xyz[0])
        pose_stamped.pose.position.y = float(xyz[1])
        pose_stamped.pose.position.z = float(xyz[2])
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        ik_req.pose_stamped = pose_stamped

        if seed_joints is not None:
            ik_req.robot_state = RobotState()
            ik_req.robot_state.joint_state = JointState()
            ik_req.robot_state.joint_state.name = list(ARM_JOINTS)
            ik_req.robot_state.joint_state.position = [float(j) for j in seed_joints]

        req.ik_request = ik_req
        fut = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None:
            return None, None
        res = fut.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, res.error_code.val

        name_to_pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [name_to_pos[j] for j in ARM_JOINTS], res.error_code.val

    def execute_joints(self, joints: Sequence[float], duration_sec: float = 2.0) -> bool:
        """Send solved joint target to arm_trajectory_controller."""
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /arm_trajectory_controller/follow_joint_trajectory unavailable')
            return False
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in joints]
        pt.velocities = [0.0] * 6
        pt.time_from_start = duration_from_seconds(max(0.2, duration_sec))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        send_fut = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def _format_pose(label: str, xyz: Sequence[float], rpy: Sequence[float], quat: Sequence[float]) -> str:
    rpy_deg = [math.degrees(a) for a in rpy]
    return (
        f'{label}:\n'
        f'  xyz  (m)  : [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]\n'
        f'  rpy  (rad): [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]\n'
        f'  rpy  (deg): [{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}]\n'
        f'  quat (xyzw): [{quat[0]:.5f}, {quat[1]:.5f}, {quat[2]:.5f}, {quat[3]:.5f}]'
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Forward & Inverse Kinematics CLI for sml_arm (Local Chain + MoveIt 2).'
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        '--fk',
        nargs=6,
        type=float,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
        help='Compute Forward Kinematics for 6 joint angles (radians).',
    )
    mode.add_argument(
        '--ik',
        nargs='+',
        type=float,
        metavar='VAL',
        help='Compute Inverse Kinematics for X Y Z [ROLL PITCH YAW] (meters, radians).',
    )
    parser.add_argument(
        '--position-only',
        action='store_true',
        help='Ignore target orientation during IK (solve 3D position X Y Z only).',
    )
    parser.add_argument(
        '--frame',
        choices=['world', 'base_link'],
        default='world',
        help='Reference frame for IK target (default: world).',
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        help='Execute solved IK joint configuration via arm_trajectory_controller.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=2.0,
        help='Execution duration in seconds when --execute is set (default: 2.0).',
    )
    parser.add_argument(
        '--no-moveit',
        action='store_true',
        help='Skip querying MoveIt 2 /compute_fk and /compute_ik services.',
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args(argv)

    client: Optional[MoveItKinematicsClient] = None
    if not args.no_moveit or args.execute:
        rclpy.init(args=ros_args)
        client = MoveItKinematicsClient()

    exit_code = 0
    try:
        if args.fk is not None:
            for ref in ('world', 'base_link'):
                res = compute_fk_local(args.fk, frame=ref)
                print(_format_pose(f'Local FK ({ref} -> ee_link)', res['xyz'], res['rpy'], res['quat']))
            if client is not None:
                for ref in ('world', 'base_link'):
                    mv = client.call_moveit_fk(args.fk, frame_id=ref)
                    if mv is not None:
                        print(_format_pose(f'MoveIt /compute_fk ({ref} -> ee_link)', *mv))
                    else:
                        print(f'MoveIt /compute_fk ({ref}): service not active (local FK used).')

        elif args.ik is not None:
            vals = args.ik
            if len(vals) == 3:
                target_xyz = vals
                target_rpy = [0.0, 0.0, 0.0]
                pos_only = True if args.position_only or True else False
            elif len(vals) == 6:
                target_xyz = vals[:3]
                target_rpy = vals[3:]
                pos_only = args.position_only
            else:
                parser.error('--ik requires 3 values (X Y Z) or 6 values (X Y Z ROLL PITCH YAW)')
                return 2

            solved_joints: Optional[List[float]] = None
            if client is not None and not pos_only:
                mv_sol, err_code = client.call_moveit_ik(
                    target_xyz, target_rpy, seed_joints=POSES['ready'], frame_id=args.frame
                )
                if mv_sol is not None:
                    solved_joints = mv_sol
                    print(f'MoveIt /compute_ik ({args.frame}) SUCCEEDED:')
                    print('  joints (rad):', [round(v, 5) for v in solved_joints])
                    print('  joints (deg):', [round(math.degrees(v), 2) for v in solved_joints])
                elif err_code is not None:
                    print(f'MoveIt /compute_ik returned error code {err_code}; trying local DLS solver.')
                else:
                    print('MoveIt /compute_ik service unavailable; using local DLS solver.')

            if solved_joints is None:
                ok, q_sol, pos_err, rot_err = compute_ik_local(
                    target_xyz,
                    target_rpy=None if pos_only else target_rpy,
                    frame=args.frame,
                    position_only=pos_only,
                )
                solved_joints = [float(v) for v in q_sol]
                status = 'CONVERGED' if ok else 'BEST_EFFORT (unreachable or tolerance exceeded)'
                print(f'Local DLS IK ({args.frame}, position_only={pos_only}) {status}:')
                print('  joints (rad):', [round(v, 5) for v in solved_joints])
                print('  joints (deg):', [round(math.degrees(v), 2) for v in solved_joints])
                print(f'  pos_err={pos_err * 1000.0:.3f} mm, rot_err={math.degrees(rot_err):.3f} deg')
                if not ok:
                    exit_code = 1

            fk_verify = compute_fk_local(solved_joints, frame=args.frame)
            print(_format_pose(f'FK Verification ({args.frame} -> ee_link)', fk_verify['xyz'], fk_verify['rpy'], fk_verify['quat']))

            if args.execute and exit_code == 0 and client is not None:
                ok_exec = client.execute_joints(solved_joints, duration_sec=args.duration)
                print('Execution:', 'SUCCEEDED' if ok_exec else 'FAILED')
                if not ok_exec:
                    exit_code = 1
    finally:
        if client is not None:
            client.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
