"""Standalone URDF/Xacro visualization launch file for sml_arm."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    model = LaunchConfiguration('model')

    pkg_share = get_package_share_directory('sml_arm_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'sml_arm.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'sml_arm.rviz')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        model,
        ' use_sim:=false use_fake_hardware:=true',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui when true, else joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot URDF/Xacro file.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),
    ])
