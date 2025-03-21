#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import re
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.command_subscription = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10)
        
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.energy_publisher = self.create_publisher(Float32, 'robot_energy', 10)
        
        self.energy = 100.0
        self.publish_energy()

        self.stop_timer = None
        
        self.get_logger().info('Control Node initialized')
    
    def publish_energy(self):
        msg = Float32()
        msg.data = float(self.energy)
        self.energy_publisher.publish(msg)
        self.get_logger().info(f"Current energy: {self.energy}")
    
    def update_energy(self, delta):
        self.energy += delta
        self.energy = max(0.0, self.energy)
        self.energy = min(100.0, self.energy)
        self.publish_energy()
    
    def get_duration(self, command):
        number_words = {
            "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
            "seven": 7, "eight": 8, "nine": 9, "ten": 10
        }
        
        match = re.search(r'(forward|backward|left|right)\s+(\d+|one|two|three|four|five|six|seven|eight|nine|ten)(?:\s*seconds?|s)?', command, re.IGNORECASE)
        
        if match:
            duration_str = match.group(2).lower()  # Extracted duration
            return int(duration_str) if duration_str.isdigit() else number_words.get(duration_str, 2)  # Convert words to numbers
        return 2

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')
        
        if "heal" in command:
            self.update_energy(25.0)
            return
        
        elif "kaboom" in command:
            self.update_energy(50.0)
            return
        
        elif "stop" in command:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_publisher.publish(twist)
            return

        if self.energy <= 0:
            self.get_logger().info("Not enough energy to move!")
            return
    
        twist = Twist()
        
        if "left" in command:
            duration = self.get_duration(command)
            twist.angular.z = 0.5
            current_energy = self.energy
            dur  = min(duration, current_energy / 2)
            self.update_energy(-2.0 * duration)
            self.publish_cmd_vel(twist, dur)
        
        elif "right" in command:
            duration = self.get_duration(command)
            twist.angular.z = -0.5
            current_energy = self.energy
            dur  = min(duration, current_energy / 2)
            self.update_energy(-2.0 * duration)
            self.publish_cmd_vel(twist, dur)
        
        elif "forward" in command:
            duration = self.get_duration(command)
            twist.linear.x = 0.2
            dur = min(duration, self.energy / 3.0)
            self.update_energy(-duration * 3.0) 
            self.publish_cmd_vel(twist, dur)

        elif "backward" in command:
            duration = self.get_duration(command)
            twist.linear.x = -0.2
            dur = min(duration, self.energy / 3.0)
            self.update_energy(-duration * 3.0)
            self.publish_cmd_vel(twist, dur)

        
    
    def publish_cmd_vel(self, twist_msg, duration):
        self.vel_publisher.publish(twist_msg)
        
        if self.stop_timer:
            self.stop_timer.cancel()
        
        self.stop_timer = self.create_timer(duration, self.stop_robot)
    
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
