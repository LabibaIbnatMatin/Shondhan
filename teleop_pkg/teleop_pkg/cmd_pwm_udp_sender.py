#!/usr/bin/env python3
"""
cmd_pwm_udp_sender.py

Subscribes to /cmd_pwm (std_msgs/Int32MultiArray) and forwards each
message as a JSON array [linear_pwm, angular_pwm] over UDP.
"""

import socket
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

DEST_IP = "192.168.2.177"
DEST_PORT = 5005  # pick any free UDP port; must match receiver


class CmdPwmUdpSender(Node):
    def __init__(self):
        super().__init__('cmd_pwm_udp_sender')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sub = self.create_subscription(
            Int32MultiArray, 'cmd_pwm', self.callback, 10
        )
        self.get_logger().info(
            f'UDP sender ready -> {DEST_IP}:{DEST_PORT}'
        )

    def callback(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('cmd_pwm message too short, skipping')
            return

        payload = [int(msg.data[0]), int(msg.data[1])]
        data = json.dumps(payload).encode('utf-8')

        try:
            self.sock.sendto(data, (DEST_IP, DEST_PORT))
            print(f"[UDP SENT] {payload}")               # visible in terminal
            self.get_logger().info(f"Sent: {payload}")    # visible in ros2 logs
        except OSError as e:
            self.get_logger().error(f"UDP send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdPwmUdpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()