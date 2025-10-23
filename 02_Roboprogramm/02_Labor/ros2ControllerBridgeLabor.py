#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 9999

# UDP Socket Setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

class UDPtoCmdVel(Node):
    def __init__(self):
        super().__init__('udp_to_cmdvel')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(f"Listening for UDP commands on port {UDP_PORT}")

        # Parameter für feste Geschwindigkeit
        self.linear_speed = 0.8
        self.angular_speed = 1.0

    def spin_once(self):
        try:
            data, addr = sock.recvfrom(1024)
            command = data.decode().strip().lower()
            self.get_logger().info(f"Received from {addr}: '{command}'")

            twist = Twist()

            if command == "up":
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            elif command == "down":
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
            elif command == "left":
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif command == "right":
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif command == "stop":
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Ungültiger Befehl -> nichts tun
                self.get_logger().warn(f"Unknown command: '{command}'")
                return

            self.pub.publish(twist)
            self.get_logger().info(f"Published cmd_vel: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

        except BlockingIOError:
            # Kein Paket empfangen -> einfach weitermachen
            pass

def main(args=None):
    rclpy.init(args=args)
    node = UDPtoCmdVel()
    try:
        while rclpy.ok():
            node.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sock.close()

if __name__ == '__main__':
    main()

