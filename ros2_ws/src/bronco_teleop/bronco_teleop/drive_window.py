#!/usr/bin/env python3
"""Drive Bronco like a video-game car, from a small window.

    ros2 run bronco_teleop drive_window

    HOLD  Space        drive (let go = stop)
    HOLD  Left / A     steer left   (let go = wheels return to center)
    HOLD  Right / D    steer right
          Up / W       raise the drive speed (+0.1 m/s per press; hold to keep raising)
          Down / S     lower it (below 0 = reverse)
          Q / Esc      quit

The window has to be selected (focused) for the keys to work. If it loses
focus while you're holding keys, the car stops (safety).

Why a window instead of the terminal? A terminal only tells a program when a
key is PRESSED (and repeats it), never when it's RELEASED, and while you hold
two keys only the last one repeats. A GUI toolkit (Qt, the same one RViz uses)
gets real press AND release events from the window system, so "hold to drive"
works, including Space + arrow together.
"""

import math
import signal
import sys

import rclpy
from geometry_msgs.msg import Twist
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from rclpy.node import Node

LEFT_KEYS = (Qt.Key_Left, Qt.Key_A)
RIGHT_KEYS = (Qt.Key_Right, Qt.Key_D)
UP_KEYS = (Qt.Key_Up, Qt.Key_W)
DOWN_KEYS = (Qt.Key_Down, Qt.Key_S)
QUIT_KEYS = (Qt.Key_Q, Qt.Key_Escape)

UPDATE_HZ = 50


class DriveNode(Node):
    """The ROS side: parameters and the /cmd_vel publisher."""

    def __init__(self):
        super().__init__('drive_window')
        self.wheelbase = self.declare_parameter('wheelbase', 0.33).value
        # Max steering angle, degrees. Must not exceed the model's steering
        # limit (steering_limit in bronco.urdf.xacro, 0.5 rad = 28.6 deg),
        # otherwise the simulator just clamps it there.
        self.max_steer = self.declare_parameter('max_steer_deg', 28.0).value
        # How fast the wheels turn toward full lock / back to center (deg/s).
        # Real steering servos take ~0.1-0.2 s to go lock to lock.
        self.steer_rate = self.declare_parameter('steer_rate_deg_s', 180.0).value
        self.max_speed = self.declare_parameter('max_speed', 3.0).value
        self.speed_step = self.declare_parameter('speed_step', 0.1).value
        self.start_speed = self.declare_parameter('start_speed', 0.5).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def send(self, speed, steer_deg):
        msg = Twist()
        msg.linear.x = speed
        # Twist carries a turn RATE, not a steering angle. Bicycle model:
        #   turn_rate = speed * tan(steering_angle) / wheelbase
        # (so at speed 0 the turn rate is 0, and the sim's wheels stay straight)
        msg.angular.z = speed * math.tan(math.radians(steer_deg)) / self.wheelbase
        self.pub.publish(msg)


class DriveWindow(QWidget):
    """The GUI side: tracks which keys are held and updates 50x per second."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.held = set()                  # 'left', 'right', 'gas'
        self.set_speed = node.start_speed  # speed used while Space is held
        self.steer = 0.0                   # current steering angle, degrees
        self.speed = 0.0                   # speed currently being sent

        self.setWindowTitle('Bronco drive')
        self.label = QLabel()
        self.label.setFont(QFont('Monospace', 12))
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)
        self.setFocusPolicy(Qt.StrongFocus)  # so this window receives keyboard input

        # A QTimer calls tick() repeatedly from Qt's event loop. That's how we
        # mix Qt (which owns the main loop) with ROS: each tick publishes a
        # command and lets ROS do any pending work with spin_once.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(1000 / UPDATE_HZ))
        self.update_label()

    # ---------------- keyboard ----------------
    def keyPressEvent(self, event):
        key = event.key()
        # Speed changes are allowed to auto-repeat: hold Up to keep raising it
        if key in UP_KEYS:
            self.set_speed = min(self.set_speed + self.node.speed_step, self.node.max_speed)
        elif key in DOWN_KEYS:
            self.set_speed = max(self.set_speed - self.node.speed_step, -self.node.max_speed)
        if abs(self.set_speed) < 1e-6:
            self.set_speed = 0.0

        # Everything else is "is it held?". The OS sends fake repeated presses
        # while a key is held; isAutoRepeat() lets us ignore those.
        if event.isAutoRepeat():
            return
        if key in LEFT_KEYS:
            self.held.add('left')
        elif key in RIGHT_KEYS:
            self.held.add('right')
        elif key == Qt.Key_Space:
            self.held.add('gas')
        elif key in QUIT_KEYS:
            self.close()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in LEFT_KEYS:
            self.held.discard('left')
        elif key in RIGHT_KEYS:
            self.held.discard('right')
        elif key == Qt.Key_Space:
            self.held.discard('gas')

    def focusOutEvent(self, event):
        # If you click away, we never get the key releases, so let go of everything
        self.held.clear()
        super().focusOutEvent(event)

    # ---------------- control loop ----------------
    def tick(self):
        dt = 1.0 / UPDATE_HZ

        # Steering target: full left, full right, or center (both/neither held)
        target = self.node.max_steer * (('left' in self.held) - ('right' in self.held))
        # Move toward the target at a limited rate, like a real servo would,
        # instead of jumping instantly
        max_change = self.node.steer_rate * dt
        self.steer += max(-max_change, min(max_change, target - self.steer))

        self.speed = self.set_speed if 'gas' in self.held else 0.0
        self.node.send(self.speed, self.steer)

        rclpy.spin_once(self.node, timeout_sec=0)
        self.update_label()

    def update_label(self):
        bar = int(round(self.steer / self.node.max_steer * 10)) if self.node.max_steer else 0
        wheel = ''.join('|' if i == -bar else '-' for i in range(-10, 11))
        gas = 'DRIVING' if 'gas' in self.held else 'stopped'
        self.label.setText(
            'HOLD Space = drive    HOLD Left/Right = steer\n'
            'Up/Down = set speed   Q = quit\n\n'
            f'Set speed : {self.set_speed:+5.2f} m/s'
            f'{"  (reverse)" if self.set_speed < 0 else ""}\n'
            f'Now       : {self.speed:+5.2f} m/s   {gas}\n'
            f'Steering  : {self.steer:+5.1f} deg\n'
            f'            L[{wheel}]R'
        )

    def closeEvent(self, event):
        self.timer.stop()
        self.node.send(0.0, 0.0)   # always leave the car stopped
        event.accept()


def main():
    rclpy.init()
    node = DriveNode()
    app = QApplication(sys.argv)
    window = DriveWindow(node)
    window.show()

    # Let Ctrl+C in the terminal close the window too
    signal.signal(signal.SIGINT, lambda *_: window.close())

    try:
        run = app.exec if hasattr(app, 'exec') else app.exec_
        run()
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
