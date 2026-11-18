#!/usr/bin/env python3
"""Command interface node for sml_arm (CLI trajectories, gripper control, and daemon bridge)."""

import argparse
import sys
import time
from typing import Dict, List, Optional

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS: List[str] = [
    'dof_joint1',
    'dof_joint2',
    'dof_joint3',
    'dof_joint4',
    'dof_joint5',
    'dof_joint6',
]

GRIPPER_MIN_WIDTH = 0.0
GRIPPER_MAX_WIDTH = 0.04

POSES: Dict[str, List[float]] = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, -0.45, 0.90, 0.0, 0.65, 0.0],
    'inspect': [0.75, -0.35, 0.85, 0.40, 0.70, -0.40],
    'pre_pick': [-0.5586, -0.3542, 1.1781, 0.0, -1.6097, -0.5586],
    'pick': [-0.5586, -0.4138, 1.4686, 0.0, -1.2596, -0.5586],
    'pre_place': [0.5586, -0.3542, 1.1781, 0.0, -1.6097, 0.5586],
    'place': [0.5586, -0.4138, 1.4686, 0.0, -1.2596, 0.5586],
}


def duration_from_seconds(seconds: float) -> Duration:
    """Convert floating-point seconds to a ROS builtin_interfaces/Duration message."""
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def parse_gripper_value(raw_value: str) -> float:
    """Parse gripper command ('open', 'close', or width in meters)."""
    normalized = raw_value.strip().lower()
    if normalized == 'open':
        return GRIPPER_MAX_WIDTH
    if normalized == 'close':
        return GRIPPER_MIN_WIDTH
    width = float(raw_value)
    return max(GRIPPER_MIN_WIDTH, min(GRIPPER_MAX_WIDTH, width))


class ArmCommander(Node):
    """ROS 2 command bridge and CLI helper for sml_arm."""

    def __init__(self) -> None:
        super().__init__('sml_arm_commander')

        self.declare_parameter('default_duration', 2.0)
        self._default_duration = float(self.get_parameter('default_duration').value)

        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_trajectory_controller/follow_joint_trajectory',
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10,
        )

        self.latest_joint_states: Dict[str, float] = {}
        self._joint_state_logged = False
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            10,
        )

        self._joint_cmd_sub = None
        self._gripper_cmd_sub = None

    def enable_daemon_topics(self) -> None:
        """Activate topic subscriptions for daemon command-bridge mode."""
        self._joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/sml_arm/joint_cmd',
            self._on_joint_cmd,
            10,
        )
        self._gripper_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/sml_arm/gripper_cmd',
            self._on_gripper_cmd,
            10,
        )
        self.get_logger().info(
            'Daemon mode active: listening on /sml_arm/joint_cmd, /sml_arm/gripper_cmd, '
            'and /joint_states'
        )

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_states[name] = pos
        if not self._joint_state_logged and self.latest_joint_states:
            self._joint_state_logged = True
            summary = ', '.join(
                f'{name}: {self.latest_joint_states[name]:.3f}'
                for name in ARM_JOINTS
                if name in self.latest_joint_states
            )
            if summary:
                self.get_logger().info(f'Receiving /joint_states -> [{summary}]')

    def _on_joint_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(ARM_JOINTS):
            self.get_logger().error(
                f'Expected {len(ARM_JOINTS)} joint positions on /sml_arm/joint_cmd, '
                f'got {len(msg.data)}'
            )
            return
        self.send_trajectory_async(list(msg.data), self._default_duration)

    def _on_gripper_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) == 1:
            val = max(GRIPPER_MIN_WIDTH, min(GRIPPER_MAX_WIDTH, float(msg.data[0])))
            positions = [val, val]
        elif len(msg.data) == 2:
            positions = [
                max(GRIPPER_MIN_WIDTH, min(GRIPPER_MAX_WIDTH, float(msg.data[0]))),
                max(GRIPPER_MIN_WIDTH, min(GRIPPER_MAX_WIDTH, float(msg.data[1]))),
            ]
        else:
            self.get_logger().error(
                f'Expected 1 or 2 values on /sml_arm/gripper_cmd, got {len(msg.data)}'
            )
            return
        self.publish_gripper(positions)

    def _build_trajectory_goal(
        self, positions: List[float], duration_sec: float
    ) -> FollowJointTrajectory.Goal:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = duration_from_seconds(max(0.1, duration_sec))
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def send_trajectory_async(self, positions: List[float], duration_sec: float) -> None:
        """Send a FollowJointTrajectory goal asynchronously (used in daemon mode)."""
        if not self._trajectory_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'Action server /arm_trajectory_controller/follow_joint_trajectory unavailable'
            )
            return

        goal = self._build_trajectory_goal(positions, duration_sec)
        self.get_logger().info(
            f'Dispatching trajectory goal {[round(p, 3) for p in positions]} '
            f'over {duration_sec:.2f}s'
        )
        future = self._trajectory_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Trajectory goal was rejected by controller')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory execution succeeded')
        else:
            self.get_logger().warn(
                f'Trajectory execution finished with error_code={result.error_code}: '
                f'{result.error_string}'
            )

    def send_trajectory_sync(self, positions: List[float], duration_sec: float) -> bool:
        """Send a FollowJointTrajectory goal and wait for completion."""
        if not self._trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Timed out waiting for /arm_trajectory_controller/follow_joint_trajectory'
            )
            return False

        goal = self._build_trajectory_goal(positions, duration_sec)
        self.get_logger().info(
            f'Sending trajectory {[round(p, 3) for p in positions]} ({duration_sec:.2f}s)'
        )
        send_future = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected by arm_trajectory_controller')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'Trajectory failed (error_code={result.error_code}): {result.error_string}'
            )
            return False

        self.get_logger().info('Trajectory complete')
        return True

    def publish_gripper(self, positions: List[float], wait_for_delivery: bool = False) -> None:
        """Publish symmetric or dual-finger gripper positions to /gripper_controller/commands."""
        msg = Float64MultiArray()
        msg.data = [float(positions[0]), float(positions[1])]
        if wait_for_delivery:
            deadline = time.monotonic() + 2.0
            while self._gripper_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
        self._gripper_pub.publish(msg)
        if wait_for_delivery:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f'Published gripper command: [{msg.data[0]:.4f}, {msg.data[1]:.4f}]'
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Command interface for sml_arm controllers.'
    )
    target_group = parser.add_mutually_exclusive_group()
    target_group.add_argument(
        '--pose',
        choices=sorted(POSES.keys()),
        help='Move arm to a named preset pose.',
    )
    target_group.add_argument(
        '--joints',
        nargs=6,
        type=float,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
        help='Move arm to 6 joint target positions in radians.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=2.5,
        help='Trajectory duration in seconds (default: 2.5).',
    )
    parser.add_argument(
        '--gripper',
        type=str,
        help="Gripper command: 'open', 'close', or finger position in meters [0.0..0.04].",
    )
    parser.add_argument(
        '--daemon',
        action='store_true',
        help='Run continuously bridging /sml_arm/joint_cmd and /sml_arm/gripper_cmd.',
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args(argv)

    if not args.daemon and args.pose is None and args.joints is None and args.gripper is None:
        parser.print_help()
        return 1

    rclpy.init(args=ros_args)
    node = ArmCommander()
    exit_code = 0

    try:
        if args.pose is not None:
            if not node.send_trajectory_sync(POSES[args.pose], args.duration):
                exit_code = 1
        elif args.joints is not None:
            if not node.send_trajectory_sync(args.joints, args.duration):
                exit_code = 1

        if args.gripper is not None:
            width = parse_gripper_value(args.gripper)
            node.publish_gripper([width, width], wait_for_delivery=not args.daemon)

        if args.daemon:
            node.enable_daemon_topics()
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
