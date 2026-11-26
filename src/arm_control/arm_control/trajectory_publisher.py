#!/usr/bin/env python3
"""Publish JointTrajectory messages on /arm_controller/joint_trajectory.

By default it cycles through a few demo poses. Custom poses (radians, flat list,
one value per joint per pose) can be given as a parameter:

  ros2 run arm_control trajectory_publisher --ros-args \
      -p poses:="[0.5, 0.3, -0.3, 0.0,  0.0, 0.0, 0.0, 0.0]" -p loop:=false
"""
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TOPIC = '/arm_controller/joint_trajectory'

DEFAULT_POSES = [
    0.0, 0.0, 0.0, 0.0,
    0.8, 0.5, -0.5, 0.3,
    -0.8, -0.5, 0.5, -0.3,
    0.0, 0.0, 0.0, 0.0,
]


def to_duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.declare_parameter('joint_names',
                               ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('poses', DEFAULT_POSES)
        self.declare_parameter('move_time', 2.0)   # seconds to reach a pose
        self.declare_parameter('period', 3.5)      # seconds between poses
        self.declare_parameter('loop', True)

        self.joints = list(self.get_parameter('joint_names').value)
        flat = list(self.get_parameter('poses').value)
        n = len(self.joints)
        if not flat or len(flat) % n != 0:
            raise ValueError(
                f'"poses" length ({len(flat)}) must be a multiple of the '
                f'number of joints ({n})')
        self.poses = [flat[i:i + n] for i in range(0, len(flat), n)]
        self.move_time = float(self.get_parameter('move_time').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.index = 0

        self.pub = self.create_publisher(JointTrajectory, TOPIC, 10)
        self.timer = self.create_timer(
            float(self.get_parameter('period').value), self.tick)
        self.get_logger().info(
            f'Publishing {len(self.poses)} pose(s) on {TOPIC} '
            f'(loop={self.loop})')

    def tick(self):
        if self.pub.get_subscription_count() == 0:
            self.get_logger().warn(
                f'No subscriber on {TOPIC} yet - waiting...',
                throttle_duration_sec=5.0)
            return

        pose = self.poses[self.index]
        msg = JointTrajectory()
        msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in pose]
        point.time_from_start = to_duration(self.move_time)
        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info(f'Pose {self.index}: {pose}')

        self.index += 1
        if self.index >= len(self.poses):
            if self.loop:
                self.index = 0
            else:
                self.timer.cancel()
                self.get_logger().info('Done.')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
