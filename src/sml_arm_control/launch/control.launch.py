"""Spawn ros2_control controllers for sml_arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_manager = LaunchConfiguration('controller_manager')
    spawn_position_controller = LaunchConfiguration('spawn_position_controller')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            controller_manager,
            '--controller-manager-timeout',
            '30',
        ],
        output='screen',
    )

    arm_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager',
            controller_manager,
            '--controller-manager-timeout',
            '30',
        ],
        output='screen',
    )

    # Spawned inactive by default because it shares joint position command interfaces
    # with arm_trajectory_controller. Activate via `ros2 control switch_controllers`.
    arm_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_position_controller',
            '--inactive',
            '--controller-manager',
            controller_manager,
            '--controller-manager-timeout',
            '30',
        ],
        condition=IfCondition(spawn_position_controller),
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            controller_manager,
            '--controller-manager-timeout',
            '30',
        ],
        output='screen',
    )

    spawn_arm_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_trajectory_controller_spawner,
                gripper_controller_spawner,
                arm_position_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_manager',
            default_value='/controller_manager',
            description='Name of the controller_manager node.',
        ),
        DeclareLaunchArgument(
            'spawn_position_controller',
            default_value='false',
            description='Load arm_position_controller in inactive state for hot-switching.',
        ),
        joint_state_broadcaster_spawner,
        spawn_arm_controllers_after_jsb,
    ])
