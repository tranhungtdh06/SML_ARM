"""Unified MoveIt 2 + Control Studio GUI bringup (supports Gazebo Harmonic or Mock Hardware)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_bringup = get_package_share_directory('sml_arm_bringup')
    pkg_moveit = get_package_share_directory('sml_arm_moveit_config')

    use_sim = LaunchConfiguration('use_sim')
    headless = LaunchConfiguration('headless')
    fast_visual = LaunchConfiguration('fast_visual')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')

    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')

    moveit_config = (
        MoveItConfigsBuilder('sml_arm', package_name='sml_arm_moveit_config')
        .robot_description(file_path=None)
        .robot_description_semantic(file_path='config/sml_arm.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )

    sim_bringup = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'sml_arm_sim.launch.py')
                ),
                launch_arguments={
                    'use_rviz': 'false',
                    'use_moveit': 'false',
                    'use_gui': 'false',
                    'headless': headless,
                    'fast_visual': fast_visual,
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(use_sim),
    )

    mock_bringup = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'sml_arm_mock.launch.py')
                ),
                launch_arguments={
                    'use_rviz': 'false',
                    'use_moveit': 'false',
                    'use_gui': 'false',
                    'fast_visual': fast_visual,
                }.items(),
            )
        ],
        scoped=True,
        condition=UnlessCondition(use_sim),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim}.items(),
    )

    moveit_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim},
        ],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    control_gui = Node(
        package='sml_arm_control',
        executable='arm_control_gui.py',
        name='sml_arm_control_gui',
        parameters=[{'use_sim_time': use_sim}],
        condition=IfCondition(use_gui),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use Gazebo Harmonic when true, mock hardware when false.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server-only without GUI when true.',
        ),
        DeclareLaunchArgument(
            'fast_visual',
            default_value='false',
            description='Use low-poly simplified visual meshes to reduce GPU load.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz2 with MoveIt MotionPlanning plugin (off by default to save GPU/RAM alongside GUI).',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start the PyQt5 Industrial Arm Control Studio GUI.',
        ),
        sim_bringup,
        mock_bringup,
        move_group,
        moveit_rviz,
        control_gui,
    ])
