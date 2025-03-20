#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class EnergyMonitorNode(Node):
    def __init__(self):
        super().__init__('energy_node')
        
        self.energy_subscription = self.create_subscription(
            Float32,
            'robot_energy',
            self.energy_callback,
            10)
        
        self.get_logger().info('Energy Monitor Node initialized')
    
    def energy_callback(self, msg):
        energy = msg.data
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Robot Energy: {energy:.1f}")
        self.get_logger().info("=" * 50)
        
        if energy < 20:
            self.get_logger().warn("WARNING: Low energy! Say 'heal' or 'kaboom' to recharge.")

def main(args=None):
    rclpy.init(args=args)
    node = EnergyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()