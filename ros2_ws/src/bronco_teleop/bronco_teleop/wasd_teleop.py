#!/usr/bin/env python3
"""Drive Bronco with WASD or the arrow keys, like a car.

    ros2 run bronco_teleop wasd_teleop

    W / Up     speed up   (each press +0.1 m/s; from reverse, slows down first)
    S / Down   slow down / reverse
    A / Left   steer left  (each press +3 degrees)
    D / Right  steer right
    C          center the steering
    Space / X  stop (speed 0, steering kept)
    Q / Ctrl+C quit (sends a stop first)

Why "press to change" instead of "hold to drive"? A terminal only tells a
program when a key is PRESSED (plus auto-repeat), never when it's released,
and it can't report two held keys at once. So, like cruise control, each press
nudges the speed or steering, and the car holds that until you change it.

How it works (the main ROS idea): this node is a PUBLISHER. Twenty times a
second it sends a geometry_msgs/Twist message on /cmd_vel:
    linear.x  = forward speed (m/s)
    angular.z = turn rate (rad/s, positive = left)
Whoever subscribes to /cmd_vel moves the robot: the Gazebo Ackermann plugin in
simulation, your ESP32 bridge on the real car. This node doesn't know or care
which one it is.
"""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# Map raw key input to actions. Arrow keys arrive as 3-character "escape
# sequences" (ESC [ A), not single characters.
KEY_ACTIONS = {
    'w': 'faster', '\x1b[A': 'faster',
    's': 'slower', '\x1b[B': 'slower',
    'a': 'left',   '\x1b[D': 'left',
    'd': 'right',  '\x1b[C': 'right',
    'c': 'center',
    ' ': 'stop', 'x': 'stop',
    'q': 'quit',
}


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')

        # Parameters: settings you can change at launch without editing code,
        # e.g. ros2 run bronco_teleop wasd_teleop --ros-args -p max_speed:=1.0
        self.wheelbase = self.declare_parameter('wheelbase', 0.33).value
        self.max_speed = self.declare_parameter('max_speed', 2.0).value
        self.max_steer = self.declare_parameter('max_steer_deg', 29.0).value
        self.speed_step = self.declare_parameter('speed_step', 0.1).value
        self.steer_step = self.declare_parameter('steer_step_deg', 3.0).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.0        # m/s
        self.steer_deg = 0.0    # front wheel angle, degrees (positive = left)

    # NOTE: don't name methods `handle`, `context`, `executor`, etc. in a Node
    # subclass: rclpy.Node already uses those names internally, and yours
    # would silently replace them.
    def apply_action(self, action):
        """Update speed/steering for one key action. Returns False to quit."""
        if action == 'faster':
            self.speed = min(self.speed + self.speed_step, self.max_speed)
        elif action == 'slower':
            self.speed = max(self.speed - self.speed_step, -self.max_speed)
        elif action == 'left':
            self.steer_deg = min(self.steer_deg + self.steer_step, self.max_steer)
        elif action == 'right':
            self.steer_deg = max(self.steer_deg - self.steer_step, -self.max_steer)
        elif action == 'center':
            self.steer_deg = 0.0
        elif action == 'stop':
            self.speed = 0.0
        elif action == 'quit':
            return False
        # Avoid tiny floating-point leftovers like 1e-17 when stepping back to 0
        if abs(self.speed) < 1e-6:
            self.speed = 0.0
        return True

    def publish(self):
        msg = Twist()
        msg.linear.x = self.speed
        # A car can't command a steering ANGLE through Twist, only a turn RATE.
        # Bicycle model: turn_rate = speed * tan(steering_angle) / wheelbase.
        # The Ackermann plugin does the reverse math to recover the angle.
        msg.angular.z = self.speed * math.tan(math.radians(self.steer_deg)) / self.wheelbase
        self.pub.publish(msg)

    def status(self):
        bar = int(round(self.steer_deg / self.max_steer * 10))
        wheel = ''.join('|' if i == -bar else '-' for i in range(-10, 11))
        return (f'\rspeed {self.speed:+5.2f} m/s   steer {self.steer_deg:+5.1f} deg  '
                f'L[{wheel}]R   ')


def split_keys(data):
    """Split a chunk of raw terminal input into keys (handles arrow sequences)."""
    keys, i = [], 0
    while i < len(data):
        if data.startswith('\x1b[', i) and i + 2 < len(data):
            keys.append(data[i:i + 3])
            i += 3
        else:
            keys.append(data[i].lower())
            i += 1
    return keys


def main():
    rclpy.init()
    node = WasdTeleop()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print(__doc__.split('Why')[0])  # show the key help from the top of this file
    try:
        # cbreak mode: deliver each keypress immediately (no Enter needed),
        # without echoing it. Ctrl+C still works.
        tty.setcbreak(fd)
        running = True
        while running and rclpy.ok():
            # Wait up to 50 ms for input, so we publish ~20 times per second
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                data = os.read(fd, 32).decode(errors='ignore')
                for key in split_keys(data):
                    action = KEY_ACTIONS.get(key)
                    if action and not node.apply_action(action):
                        running = False
            node.publish()
            sys.stdout.write(node.status())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave the robot stopped and the terminal back to normal
        node.speed = 0.0
        node.publish()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print('\nStopped.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
