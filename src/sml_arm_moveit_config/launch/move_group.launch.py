"""Launch MoveIt 2 move_group node for sml_arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    """Construct MoveItConfigs for sml_arm."""
    return (
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


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    moveit_config = build_moveit_config()

    move_group_params = [
        moveit_config.to_dict(),
        {
            'use_sim_time': use_sim_time,
            'publish_robot_description_semantic': True,
            'allow_trajectory_execution': True,
            'capabilities': (
                'pilz_industrial_motion_planner/MoveGroupSequenceAction '
                'pilz_industrial_motion_planner/MoveGroupSequenceService'
            ),
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'monitor_dynamics': False,
        },
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation /clock time when running with Gazebo Harmonic.',
        ),
        move_group_node,
    ])
