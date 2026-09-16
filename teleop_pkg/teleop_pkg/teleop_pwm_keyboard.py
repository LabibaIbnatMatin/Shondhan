#!/usr/bin/env python3
"""
teleop_pwm_keyboard.py

WASD -> PWM-style teleop for a search-and-rescue rover.

Mapping:
    W : forward   -> linear_pwm  = 2000
    S : backward  -> linear_pwm  = 1000
    A : left      -> angular_pwm = 2000
    D : right     -> angular_pwm = 1000
    (nothing held) -> 1500 on the relevant axis (neutral / stop)

Publishes std_msgs/Int32MultiArray on /cmd_pwm as [linear_pwm, angular_pwm].
Downstream (e.g. a serial bridge on the Teensy) should read this topic and
mix linear/angular into left/right Sabertooth motor commands.

HOLD-TO-MOVE NOTE:
A raw terminal only reports key *presses* (repeated automatically by the OS
while a key is held down); it never reports key *release*. So "hold to move"
is implemented via a timeout: each axis stays pinned to its active value as
long as repeat keystrokes keep arriving inside KEY_TIMEOUT seconds of each
other. If no repeat arrives in time, we treat the key as released and snap
that axis back to 1500. Tune KEY_TIMEOUT to your terminal's OS key-repeat
rate (0.3s is safe for most default repeat rates; lower it for snappier
release detection if your repeat rate is fast).
"""

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

NEUTRAL = 1500
FORWARD_LEFT = 2000   # W / A
BACKWARD_RIGHT = 1000  # S / D

KEY_TIMEOUT = 0.3   # seconds - no repeat within this window == key released
LOOP_HZ = 20.0       # publish / poll rate

# Slew-rate limits, in PWM-units/second, applied to each axis independently.
# ACCEL: how fast the axis moves AWAY from neutral (i.e. speeding up).
# DECEL: how fast it moves BACK TOWARD neutral (i.e. slowing down / stopping).
# DECEL > ACCEL here so releasing a key (or reversing direction) brakes
# faster than the rover spins up - tune both to your rover's mass/traction.
ACCEL_RATE = 1000.0   # units/sec -> ~0.5s to go from neutral to full (500 units)
DECEL_RATE = 2000.0   # units/sec -> ~0.25s to go from full back to neutral

INSTRUCTIONS = """
WASD teleop (hold-to-move)
---------------------------
  W : forward     A : left
  S : backward    D : right

Release a key -> that axis returns to neutral (1500).
CTRL-C to quit (publishes neutral once before exiting).
---------------------------
"""


class TeleopPWMKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_pwm_keyboard')
        self.pub = self.create_publisher(Int32MultiArray, 'cmd_pwm', 10)

        # self.linear / self.angular are the values actually published, and
        # move gradually toward target_linear / target_angular each tick.
        self.linear = NEUTRAL
        self.angular = NEUTRAL
        self.target_linear = NEUTRAL
        self.target_angular = NEUTRAL

        now = time.time()
        self.last_w = 0.0
        self.last_s = 0.0
        self.last_a = 0.0
        self.last_d = 0.0

        # Put the terminal into raw mode so we can read single keystrokes
        # without waiting for Enter, and without local echo.
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.timer = self.create_timer(1.0 / LOOP_HZ, self.loop)
        self.get_logger().info('teleop_pwm_keyboard started. Press CTRL-C to quit.')

    def get_keys_nonblocking(self):
        """Return a list of all buffered keystrokes available right now."""
        keys = []
        while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not rlist:
                    break
                keys.append(sys.stdin.read(1))
                
        return keys
    def loop(self):
        now = time.time()
        keys = self.get_keys_nonblocking()

        for key in keys:
            k = key.lower()
            if k == 'w':
                self.target_linear = FORWARD_LEFT
                self.last_w = now
            elif k == 's':
                self.target_linear = BACKWARD_RIGHT
                self.last_s = now
            elif k == 'a':
                self.target_angular = FORWARD_LEFT
                self.last_a = now
            elif k == 'd':
                self.target_angular = BACKWARD_RIGHT
                self.last_d = now
            elif key == '\x03':
                raise KeyboardInterrupt
        # Release detection: if the axis's target is pinned away from neutral
        # but its driving key hasn't repeated within KEY_TIMEOUT, the key has
        # been released -> retarget to neutral (the ramp below then eases the
        # PUBLISHED value back down at DECEL_RATE, rather than snapping it).
        if self.target_linear == FORWARD_LEFT and now - self.last_w > KEY_TIMEOUT:
            self.target_linear = NEUTRAL
        if self.target_linear == BACKWARD_RIGHT and now - self.last_s > KEY_TIMEOUT:
            self.target_linear = NEUTRAL

        if self.target_angular == FORWARD_LEFT and now - self.last_a > KEY_TIMEOUT:
            self.target_angular = NEUTRAL
        if self.target_angular == BACKWARD_RIGHT and now - self.last_d > KEY_TIMEOUT:
            self.target_angular = NEUTRAL

        dt = 1.0 / LOOP_HZ
        self.linear = self._ramp(self.linear, self.target_linear, dt)
        self.angular = self._ramp(self.angular, self.target_angular, dt)

        msg = Int32MultiArray()
        msg.data = [int(round(self.linear)), int(round(self.angular))]
        self.pub.publish(msg)

    @staticmethod
    def _ramp(current, target, dt):
        """Move `current` toward `target` by at most one tick's worth of
        slew, using ACCEL_RATE when moving away from neutral and the
        (faster) DECEL_RATE when moving back toward it."""
        diff = target - current
        if diff == 0:
            return current
        moving_away_from_neutral = abs(target - NEUTRAL) > abs(current - NEUTRAL)
        rate = ACCEL_RATE if moving_away_from_neutral else DECEL_RATE
        max_step = rate * dt
        if abs(diff) <= max_step:
            return target
        return current + max_step * (1.0 if diff > 0 else -1.0)

    def publish_neutral_and_restore_terminal(self):
        msg = Int32MultiArray()
        msg.data = [NEUTRAL, NEUTRAL]
        self.pub.publish(msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPWMKeyboard()
    print(INSTRUCTIONS)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_neutral_and_restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
