"""Lightweight mock/fake hardware ros2_control + RViz2 bringup for sml_arm (no Gazebo)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('sml_arm_description')
    pkg_control = get_package_share_directory('sml_arm_control')
    pkg_moveit = get_package_share_directory('sml_arm_moveit_config')

    xacro_file = os.path.join(pkg_description, 'urdf', 'sml_arm.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'sml_arm.rviz')
    controllers_yaml = os.path.join(pkg_control, 'config', 'sml_arm_controllers.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    fast_visual = LaunchConfiguration('fast_visual')
    use_moveit = LaunchConfiguration('use_moveit')
    use_gui = LaunchConfiguration('use_gui')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' use_sim:=false use_fake_hardware:=true fast_visual:=',
        fast_visual,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'publish_frequency': 30.0,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    arm_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    spawn_arm_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_trajectory_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(use_moveit),
    )

    control_gui = Node(
        package='sml_arm_control',
        executable='arm_control_gui.py',
        name='sml_arm_control_gui',
        condition=IfCondition(use_gui),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 alongside mock hardware controller_manager.',
        ),
        DeclareLaunchArgument(
            'fast_visual',
            default_value='false',
            description='Use low-poly simplified visual meshes to reduce GPU load.',
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='false',
            description='Launch MoveIt 2 move_group node alongside mock hardware.',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Launch PyQt5 Industrial Arm Control Studio GUI.',
        ),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        spawn_arm_controllers_after_jsb,
        rviz2,
        move_group,
        control_gui,
    ])
