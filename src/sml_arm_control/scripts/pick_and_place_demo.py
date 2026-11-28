#!/usr/bin/env python3
"""Automated Pick-and-Place demonstration node for sml_arm with parallel gripper."""

import argparse
import os
import sys
import time
from typing import List, Optional, Sequence

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_kinematics import ARM_JOINTS, POSES, compute_segment_duration, duration_from_seconds  # noqa: E402


GRIPPER_OPEN = 0.038
GRIPPER_GRASP = 0.0155


class PickAndPlaceDemo(Node):
    """Executes coordinated arm trajectories and gripper commands for pick-and-place."""

    def __init__(self) -> None:
        super().__init__('sml_arm_pick_and_place_demo')
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_trajectory_controller/follow_joint_trajectory',
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10,
        )

    def wait_ready(self, timeout_sec: float = 20.0) -> bool:
        self.get_logger().info('Waiting for arm_trajectory_controller and gripper_controller...')
        if not self._traj_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('arm_trajectory_controller action server unavailable.')
            return False
        deadline = time.monotonic() + timeout_sec
        while self._gripper_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._gripper_pub.get_subscription_count() == 0:
            self.get_logger().warn('No subscriber detected on /gripper_controller/commands yet; continuing.')
        return True

    def move_arm(self, pose_name: str, positions: Sequence[float], duration_sec: float) -> bool:
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        pt.time_from_start = duration_from_seconds(max(0.2, duration_sec))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f'Arm -> {pose_name} ({duration_sec:.2f}s)')
        send_fut = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Goal rejected for {pose_name}')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Failed moving to {pose_name}: error_code={res.error_code}')
            return False
        return True

    def set_gripper(self, width_m: float, settle_sec: float) -> None:
        label = 'OPEN' if width_m > 0.025 else 'GRASP'
        self.get_logger().info(f'Gripper -> {label} ({width_m * 1000.0:.1f} mm/finger)')
        msg = Float64MultiArray()
        msg.data = [float(width_m), float(width_m)]
        self._gripper_pub.publish(msg)
        end = time.monotonic() + max(0.05, settle_sec)
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_cycle(self, forward: bool, speed_scale: float) -> bool:
        s = max(0.2, speed_scale)
        src_pre = 'pre_pick' if forward else 'pre_place'
        src_grasp = 'pick' if forward else 'place'
        dst_pre = 'pre_place' if forward else 'pre_pick'
        dst_drop = 'place' if forward else 'pick'

        steps = [
            ('gripper', GRIPPER_OPEN, 0.35 / s),
            ('arm', 'ready', None),
            ('arm', src_pre, None),
            ('arm', src_grasp, None),
            ('gripper', GRIPPER_GRASP, 0.6 / s),
            ('arm', src_pre, None),
            ('arm', dst_pre, None),
            ('arm', dst_drop, None),
            ('gripper', GRIPPER_OPEN, 0.5 / s),
            ('arm', dst_pre, None),
            ('arm', 'ready', None),
        ]

        prev_q = POSES['home'] if forward else POSES['ready']
        for kind, target, dur in steps:
            if kind == 'gripper':
                self.set_gripper(float(target), float(dur))
            else:
                q_target = POSES[str(target)]
                seg_dur = compute_segment_duration(prev_q, q_target, speed_scale=s)
                if not self.move_arm(str(target), q_target, seg_dur):
                    return False
                prev_q = q_target
        return True


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description='Pick-and-Place demo for sml_arm.')
    parser.add_argument(
        '--loop',
        action='store_true',
        help='Continuously pick and place back and forth between pick_table and place_table.',
    )
    parser.add_argument(
        '--speed-scale',
        type=float,
        default=1.0,
        help='Motion speed multiplier (>1.0 runs faster, default: 1.0).',
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = PickAndPlaceDemo()
    exit_code = 0

    try:
        if not node.wait_ready():
            return 1

        forward = True
        cycle_num = 1
        while rclpy.ok():
            direction = 'pick_table -> place_table' if forward else 'place_table -> pick_table'
            node.get_logger().info(f'=== Starting Pick & Place Cycle {cycle_num} ({direction}) ===')
            if not node.run_cycle(forward=forward, speed_scale=args.speed_scale):
                exit_code = 1
                break
            node.get_logger().info(f'=== Completed Pick & Place Cycle {cycle_num} ===')
            if not args.loop:
                break
            forward = not forward
            cycle_num += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
