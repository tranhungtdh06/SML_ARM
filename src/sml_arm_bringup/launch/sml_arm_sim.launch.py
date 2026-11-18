"""Full Gazebo Harmonic + ros2_control + RViz2 simulation bringup for sml_arm."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('sml_arm_description')
    pkg_control = get_package_share_directory('sml_arm_control')
    pkg_gazebo = get_package_share_directory('sml_arm_gazebo')
    pkg_moveit = get_package_share_directory('sml_arm_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(pkg_gazebo, 'worlds', 'empty_arm_world.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'sml_arm.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'sml_arm.rviz')
    controllers_yaml = os.path.join(pkg_control, 'config', 'sml_arm_controllers.yaml')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_bridge.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    fast_visual = LaunchConfiguration('fast_visual')
    use_moveit = LaunchConfiguration('use_moveit')
    use_gui = LaunchConfiguration('use_gui')

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
        ],
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

    spawn_controllers_on_entity_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
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
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_moveit),
    )

    control_gui = Node(
        package='sml_arm_control',
        executable='arm_control_gui.py',
        name='sml_arm_control_gui',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_gui),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 alongside Gazebo Harmonic.',
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
            'world',
            default_value=default_world,
            description='Absolute path to Gazebo Harmonic SDF world file.',
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='false',
            description='Launch MoveIt 2 move_group node alongside simulation.',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Launch PyQt5 Industrial Arm Control Studio GUI.',
        ),
        robot_state_publisher,
        gz_sim,
        gz_bridge,
        gz_spawn_entity,
        spawn_controllers_on_entity_spawn,
        rviz2,
        move_group,
        control_gui,
    ])
