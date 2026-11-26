#!/usr/bin/env python3
"""Tkinter slider GUI that publishes JointTrajectory on
/arm_controller/joint_trajectory.

Move a slider -> one trajectory is published -> the same message moves the arm
in Gazebo, in RViz2 and on the real servos (via arm_hardware).
"""
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TOPIC = '/arm_controller/joint_trajectory'
PUBLISH_PERIOD_MS = 50  # GUI tick: max 20 messages per second


class SliderPublisher(Node):
    def __init__(self):
        super().__init__('slider_gui')
        self.declare_parameter('joint_names',
                               ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('lower_limit', -1.5708)
        self.declare_parameter('upper_limit', 1.5708)
        self.declare_parameter('move_time', 0.3)

        self.joints = list(self.get_parameter('joint_names').value)
        self.lower = float(self.get_parameter('lower_limit').value)
        self.upper = float(self.get_parameter('upper_limit').value)
        self.move_time = float(self.get_parameter('move_time').value)
        self.pub = self.create_publisher(JointTrajectory, TOPIC, 10)

    def send(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        sec = int(self.move_time)
        point.time_from_start = Duration(
            sec=sec, nanosec=int(round((self.move_time - sec) * 1e9)))
        msg.points = [point]
        self.pub.publish(msg)


class App:
    def __init__(self, node: SliderPublisher):
        self.node = node
        self.dirty = False

        self.root = tk.Tk()
        self.root.title('Arm control - /arm_controller/joint_trajectory')
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky='nsew')

        self.vars = []
        self.labels = []
        for row, name in enumerate(node.joints):
            ttk.Label(frame, text=name, width=8).grid(row=row, column=0, sticky='w')
            var = tk.DoubleVar(value=0.0)
            scale = tk.Scale(
                frame, variable=var, from_=node.lower, to=node.upper,
                resolution=0.01, orient=tk.HORIZONTAL, length=380,
                showvalue=False, command=self.on_change)
            scale.grid(row=row, column=1, padx=8)
            label = ttk.Label(frame, text='', width=18)
            label.grid(row=row, column=2, sticky='w')
            self.vars.append(var)
            self.labels.append(label)

        ttk.Button(frame, text='Reset (all 0)', command=self.reset).grid(
            row=len(node.joints), column=1, pady=(10, 0))

        self.update_labels()
        self.root.after(PUBLISH_PERIOD_MS, self.tick)

    def positions(self):
        return [v.get() for v in self.vars]

    def update_labels(self):
        for var, label in zip(self.vars, self.labels):
            rad = var.get()
            label.config(text=f'{rad:+.2f} rad ({math.degrees(rad):+.0f} deg)')

    def on_change(self, _value=None):
        self.update_labels()
        self.dirty = True

    def reset(self):
        for var in self.vars:
            var.set(0.0)
        self.on_change()

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.dirty:
            self.node.send(self.positions())
            self.dirty = False
        self.root.after(PUBLISH_PERIOD_MS, self.tick)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = SliderPublisher()
    try:
        App(node).run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
