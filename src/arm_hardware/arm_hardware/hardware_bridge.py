#!/usr/bin/env python3
"""ROS 2 -> microcontroller bridge.

Subscribes to /arm_controller/joint_trajectory, interpolates the trajectory,
converts joint angles (rad) to servo angles (deg, clamped) and sends them over
USB serial as ``A,<deg1>,<deg2>\\n`` to the Arduino firmware.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from arm_hardware.servo_logic import (TrajectoryPlayer, format_command,
                                      rad_to_servo_deg)

TOPIC = '/arm_controller/joint_trajectory'


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        # One entry per servo: servo i is driven by joint_names[i].
        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('offsets_deg', [90.0, 90.0])  # servo angle at 0 rad
        self.declare_parameter('directions', [1.0, 1.0])     # +1 or -1
        self.declare_parameter('min_deg', [0.0, 0.0])
        self.declare_parameter('max_deg', [180.0, 180.0])
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('startup_delay', 2.0)  # Arduino resets on open
        self.declare_parameter('dry_run', False)      # print instead of writing

        p = self.get_parameter
        self.joints = list(p('joint_names').value)
        n = len(self.joints)
        self.offsets = list(p('offsets_deg').value)
        self.directions = list(p('directions').value)
        self.min_deg = list(p('min_deg').value)
        self.max_deg = list(p('max_deg').value)
        for name in ('offsets_deg', 'directions', 'min_deg', 'max_deg'):
            if len(p(name).value) != n:
                raise ValueError(f'"{name}" must have {n} entries (one per joint)')

        self.dry_run = bool(p('dry_run').value)
        self.serial = None
        if not self.dry_run:
            self.serial = self._open_serial(
                str(p('port').value), int(p('baudrate').value),
                float(p('startup_delay').value))
        else:
            self.get_logger().warn('dry_run=true: nothing is sent to hardware')

        self.player = TrajectoryPlayer(self.joints)
        self.last_sent = None

        self.create_subscription(JointTrajectory, TOPIC, self.on_trajectory, 10)
        self.create_timer(1.0 / float(p('rate_hz').value), self.on_timer)
        self.get_logger().info(
            f'Listening on {TOPIC}; servo order = {self.joints}')

    # ------------------------------------------------------------------
    def _open_serial(self, port, baud, delay):
        try:
            import serial  # pyserial
        except ImportError as exc:
            raise RuntimeError(
                'pyserial is missing: sudo apt install python3-serial') from exc
        try:
            ser = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as exc:
            raise RuntimeError(
                f'Cannot open {port}: {exc}. Check the cable/port, your '
                f'"dialout" group membership, or run with dry_run:=true') from exc
        self.get_logger().info(f'Opened {port} @ {baud}; waiting {delay}s for board reset')
        time.sleep(delay)
        ser.reset_input_buffer()
        return ser

    def on_trajectory(self, msg: JointTrajectory):
        points = [
            (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
             list(pt.positions))
            for pt in msg.points
        ]
        used = self.player.load(msg.joint_names, points, time.monotonic())
        if used == 0:
            self.get_logger().warn(
                f'Ignored message: none of {list(msg.joint_names)} match '
                f'{self.joints}')

    def on_timer(self):
        current = self.player.step(time.monotonic())
        angles = [
            rad_to_servo_deg(current[j], self.offsets[i], self.directions[i],
                             self.min_deg[i], self.max_deg[i])
            for i, j in enumerate(self.joints)
        ]
        line = format_command(angles)
        if line == self.last_sent:
            return  # nothing changed at 1-degree resolution
        self.last_sent = line
        self._write(line)

    def _write(self, line: str):
        if self.dry_run:
            self.get_logger().info(f'[dry_run] {line.strip()}')
            return
        try:
            self.serial.write(line.encode('ascii'))
            if self.serial.in_waiting:  # drain firmware replies (e.g. READY/ERR)
                reply = self.serial.read(self.serial.in_waiting).decode(
                    'ascii', errors='replace').strip()
                if reply:
                    self.get_logger().info(f'board: {reply}')
        except Exception as exc:  # serial unplugged etc.
            self.get_logger().error(f'Serial write failed: {exc}',
                                    throttle_duration_sec=2.0)

    def close(self):
        if self.serial is not None:
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
