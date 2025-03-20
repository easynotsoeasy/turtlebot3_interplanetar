#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')
        self.command_publisher = self.create_publisher(String, 'voice_command', 10)
        self.get_logger().info('Speaker Node initialized')
        self.get_logger().info('Type commands and press Enter')
        
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
    def input_loop(self):
        while rclpy.ok():
            try:
                text = input("Enter command: ")
                if text:
                    self.get_logger().info(f"Command entered: {text}")
                    msg = String()
                    msg.data = text.lower()
                    self.command_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error in input processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
