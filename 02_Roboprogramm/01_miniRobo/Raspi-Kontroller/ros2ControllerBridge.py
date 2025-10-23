#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 9999

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

class UDPtoROS(Node):
    def __init__(self):
        super().__init__('udp_to_ros')
        self.pub = self.create_publisher(String, 'buttons', 10)
        self.get_logger().info(f"Listening for UDP on port {UDP_PORT}")

    def spin_once(self):
        try:
            data, addr = sock.recvfrom(1024)
            msg = String()
            msg.data = data.decode()
            self.pub.publish(msg)
            self.get_logger().info(f"Received from {addr}: {msg.data}")
        except BlockingIOError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = UDPtoROS()
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

