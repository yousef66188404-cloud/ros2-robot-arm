"""Start the ROS 2 -> microcontroller bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('arm_hardware'), 'config', 'hardware.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='Print commands instead of using serial'),
        Node(
            package='arm_hardware',
            executable='hardware_bridge',
            name='hardware_bridge',
            parameters=[
                config,
                {'port': LaunchConfiguration('port'),
                 'dry_run': ParameterValue(LaunchConfiguration('dry_run'),
                                           value_type=bool)},
            ],
            output='screen',
        ),
    ])
