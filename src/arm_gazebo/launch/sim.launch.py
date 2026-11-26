"""Gazebo (gz sim) + ros2_control + RViz2.

Data flow
---------
/arm_controller/joint_trajectory --> JointTrajectoryController --> Gazebo joints
Gazebo joints --> joint_state_broadcaster --> /joint_states --> robot_state_publisher
                                                          \\--> RViz2 (same motion)
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare('arm_description')
    gz_share = FindPackageShare('arm_gazebo')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    xacro_file = PathJoinSubstitution([desc_share, 'urdf', 'arm.urdf.xacro'])
    controllers = PathJoinSubstitution([gz_share, 'config', 'controllers.yaml'])
    world = PathJoinSubstitution([gz_share, 'worlds', 'arm_world.sdf'])
    rviz_config = PathJoinSubstitution([desc_share, 'rviz', 'arm.rviz'])

    use_rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file,
                 ' sim:=true controllers_file:=', controllers]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])),
        # headless:=true runs only the Gazebo server (-s): no 3D window.
        launch_arguments={'gz_args': [
            '-r ',
            PythonExpression(["'-s ' if '", LaunchConfiguration('headless'),
                              "' == 'true' else ''"]),
            world]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'arm',
                   '-allow_renaming', 'true'],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without its GUI window'),
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        spawn_arm,
        # Start controllers only after the model exists in Gazebo.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_arm, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[arm_controller])),
        rviz,
    ])
