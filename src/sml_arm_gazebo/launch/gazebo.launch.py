"""Launch Gazebo Harmonic simulation with sml_arm spawned and clock bridged."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('sml_arm_description')
    pkg_control = get_package_share_directory('sml_arm_control')
    pkg_gazebo = get_package_share_directory('sml_arm_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(pkg_gazebo, 'worlds', 'empty_arm_world.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'sml_arm.urdf.xacro')
    controllers_yaml = os.path.join(pkg_control, 'config', 'sml_arm_controllers.yaml')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_bridge.yaml')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    fast_visual = LaunchConfiguration('fast_visual')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' use_sim:=true use_fake_hardware:=false fast_visual:=',
        fast_visual,
        ' sim_controllers_yaml:=',
        controllers_yaml,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': True,
        'publish_frequency': 30.0,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    gz_args = PythonExpression([
        "'-r -s -v 1 ' + '",
        world,
        "' if '",
        headless,
        "' == 'true' else '-r -v 1 ' + '",
        world,
        "'",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'sml_arm',
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to Gazebo Harmonic SDF world file.',
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
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X coordinate (m).'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y coordinate (m).'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z coordinate (m).'),
        robot_state_publisher,
        gz_sim,
        gz_bridge,
        gz_spawn_entity,
    ])
