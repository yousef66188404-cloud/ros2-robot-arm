"""Full pipeline: Gazebo + RViz2 + slider GUI + (optional) real servos.

  ros2 launch arm_bringup full.launch.py port:=/dev/ttyUSB0
  ros2 launch arm_bringup full.launch.py hardware:=false     # simulation only
  ros2 launch arm_bringup full.launch.py dry_run:=true       # no board attached
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('arm_gazebo'), 'launch', 'sim.launch.py'])),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('arm_hardware'), 'launch', 'hardware.launch.py'])),
        launch_arguments={'port': LaunchConfiguration('port'),
                          'dry_run': LaunchConfiguration('dry_run')}.items(),
        condition=IfCondition(LaunchConfiguration('hardware')),
    )

    gui = Node(
        package='arm_control',
        executable='slider_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('hardware', default_value='true',
                              description='Start the serial hardware bridge'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='Bridge prints instead of using serial'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the slider GUI'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim,
        hardware,
        gui,
    ])
