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
    
    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')
        
        if "heal" in command:
            self.update_energy(25.0)
            return
        elif "kaboom" in command:
            self.update_energy(50.0)
            return
    
        if self.energy <= 0:
            self.get_logger().info("Not enough energy to move!")
            return
    
        
        twist = Twist()
        
        if "left" in command:
            match = re.search(r'left\s+(\d+)s', command)
            if match:
                duration = int(match.group(1))
            else:
                duration = 2

            twist.angular.z = 0.5
            self.update_energy(-5.0)
            self.publish_cmd_vel(twist, 2.0)
        
        elif "right" in command:

            match = re.search(r'right\s+(\d+)s', command)
            if match:
                duration = int(match.group(1))
            else:
                duration = 2
            

            twist.angular.z = -0.5
            self.update_energy(-5.0)
            self.publish_cmd_vel(twist, 2.0)
        
        elif "forward" in command:
            match = re.search(r'forward\s+(\d+)s', command)
            if match:
                duration = int(match.group(1))
            else:
                duration = 2 
            
            twist.linear.x = 0.2
            self.update_energy(-duration * 3.0) 
            self.publish_cmd_vel(twist, duration)

        elif "backward" in command:
            match = re.search(r'backward\s+(\d+)s', command)
            if match:
                duration = int(match.group(1))
            else:
                duration = 2
            
            twist.linear.x = -0.2
            self.update_energy(-duration * 3.0)
            self.publish_cmd_vel(twist, duration)

        elif "stop" in command:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_publisher.publish(twist)
    
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
