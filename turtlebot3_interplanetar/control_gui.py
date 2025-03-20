#!/usr/bin/env python3

import tkinter as tk
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import time
import threading
from nav_msgs.msg import Odometry
import tf_transformations
import math

GRID_SIZE = 20  
CELL_SIZE = 20  
BOT_RADIUS = 5 

canvas = None
bot_dot = None
odom_label = None

def update_position_label(text):
    """Update the Tkinter label with odometry data."""
    global odom_label
    if odom_label is not None:
        odom_label.config(text=text)

def update_bot_position(x, y):
    """Update the position of the bot dot on the canvas."""
    global canvas, bot_dot
    center_x = CELL_SIZE * GRID_SIZE / 2 + x * CELL_SIZE
    center_y = CELL_SIZE * GRID_SIZE / 2 - y * CELL_SIZE
    if canvas is not None and bot_dot is not None:
        canvas.coords(bot_dot, center_x - BOT_RADIUS, center_y + BOT_RADIUS,
                      center_x + BOT_RADIUS, center_y - BOT_RADIUS)

class ControlNode(Node):
    def __init__(self):
        super().__init__("turtlebot3_control")
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.stop_timer = None
        self.rotation_done = threading.Event()
        self.rotation_done.set()

    def publish_cmd_vel(self, twist_msg, duration):
        self.pub.publish(twist_msg)
        if self.stop_timer:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(duration, self.stop_robot)
    
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None
        self.rotation_done.set()

    def forward(self, duration=2.0, linear_velocity=0.5): 
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = 0.0
        self.publish_cmd_vel(twist, duration)
        print("Forward Duration")

    def backward(self, duration=2.0, linear_velocity=0.5):
        twist = Twist()
        twist.linear.x = -linear_velocity
        twist.angular.z = 0.0
        self.publish_cmd_vel(twist, duration)
        print("Backward Duration")
    
    def left(self, duration=2.0):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.publish_cmd_vel(twist, duration)
        print("Left Duration")
    
    def right(self, duration=2.0):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        self.publish_cmd_vel(twist, duration)
        print("Right Duration")

    # Continuous motion commands
    def forward_cont(self, linear_velocity):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = 0.0
        self.pub.publish(twist)
        print("Forward")

    def backward_cont(self, linear_velocity):
        twist = Twist()
        twist.linear.x = -linear_velocity
        twist.angular.z = 0.0
        self.pub.publish(twist)
        print("Backward")

    def left_cont(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.pub.publish(twist)
        print("Left")

    def right_cont(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        self.pub.publish(twist)
        print("Right")

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        print("Stop")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        quaternion = (q_x, q_y, q_z, q_w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        text = f"Position: x={x:.2f}, y={y:.2f}"
        text += f"\nOrientation: roll={euler[0]:.2f}, pitch={euler[1]:.2f}, yaw={euler[2]:.2f}"
        
        update_position_label(text)
        
        minimap_x = float(x)
        minimap_y = float(y)
        update_bot_position(minimap_x, minimap_y)

        self.current_x = x
        self.current_y = y
        self.current_yaw = euler[2]

    def calculate_angle(self, x, y):
        dx = x - self.current_x
        dy = y - self.current_y
        angle = math.atan2(dy, dx)
        print(f"Angle: {angle}")
        return angle
    
    def calculate_distance(self, x, y):
        dx = x - self.current_x
        dy = y - self.current_y
        return math.sqrt(dx**2 + dy**2)
    
    def rotate_bot(self, x, y):
        self.rotation_done.clear()
        angle = self.calculate_angle(x, y)
        angle_diff = angle - self.current_yaw
        duration = abs(angle_diff) / 0.5
        if angle_diff > 0:
            self.left(duration)
        else:
            self.right(duration)
    
    def move_forward(self, x, y, linear_velocity=0.2):
        distance = self.calculate_distance(x, y)
        duration = distance / linear_velocity
        self.forward(duration, linear_velocity)

    def move_bot(self, x, y, linear_velocity=0.2):
        self.rotate_bot(x, y)
        self.rotation_done.wait()
        self.move_forward(x, y, linear_velocity)

def ros_spin(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    global canvas, bot_dot, odom_label  

    rclpy.init()
    node = ControlNode()

    root = tk.Tk()
    root.title("Turtlebot3 Control")
    root.geometry("800x800")

    def on_press_forward(event):
        linear_velocity = velocity_slider.get()
        node.forward_cont(linear_velocity)

    def on_press_backward(event):
        linear_velocity = velocity_slider.get()
        node.backward_cont(linear_velocity)

    def on_press_left(event):
        node.left_cont()

    def on_press_right(event):
        node.right_cont()

    def on_release(event):
        node.stop()

    def btn_stop_click():
        node.stop()

    def btn_forward_click():
        node.forward()

    def btn_backward_click():
        node.backward()

    def btn_left_click():
        node.left()

    def btn_right_click():
        node.right()

    def on_click(event):
        x, y = event.x, event.y
        print(f"Pixel coordinates: ({x}, {y})")
        grid_x = x / CELL_SIZE
        grid_y = y / CELL_SIZE
        map_x = grid_x - GRID_SIZE / 2
        map_y = GRID_SIZE / 2 - grid_y
        print(f"Map coordinates: ({map_x}, {map_y})")
        threading.Thread(target=node.move_bot, args=(map_x, map_y, velocity_slider.get())).start()


    velocity_slider = tk.Scale(root, from_=0.0, to=1.0, resolution=0.01, orient="horizontal", label="Velocity")
    velocity_slider.set(0.2)

    canvas = tk.Canvas(root, width=GRID_SIZE * CELL_SIZE, height=GRID_SIZE * CELL_SIZE, bg="white")
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            x1, y1 = i * CELL_SIZE, j * CELL_SIZE
            x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE
            canvas.create_rectangle(x1, y1, x2, y2, outline="black")
    bot_dot = canvas.create_oval(
        GRID_SIZE*CELL_SIZE / 2 - BOT_RADIUS, GRID_SIZE*CELL_SIZE / 2 + BOT_RADIUS,
        GRID_SIZE*CELL_SIZE / 2 + BOT_RADIUS, GRID_SIZE*CELL_SIZE / 2 - BOT_RADIUS,
        fill="red"
    )
    canvas.bind("<Button-1>", on_click)

    odom_label = tk.Label(root, text="Position: x=0.00, y=0.00")

    btn_forward = tk.Button(root, text="Forward")
    btn_forward.bind("<ButtonPress-1>", on_press_forward)
    btn_forward.bind("<ButtonRelease-1>", on_release)

    btn_backward = tk.Button(root, text="Backward")
    btn_backward.bind("<ButtonPress-1>", on_press_backward)
    btn_backward.bind("<ButtonRelease-1>", on_release)

    btn_left = tk.Button(root, text="Left")
    btn_left.bind("<ButtonPress-1>", on_press_left)
    btn_left.bind("<ButtonRelease-1>", on_release)

    btn_right = tk.Button(root, text="Right")
    btn_right.bind("<ButtonPress-1>", on_press_right)
    btn_right.bind("<ButtonRelease-1>", on_release)

    btn_stop = tk.Button(root, text="Stop", command=btn_stop_click)
    btn_forward_2s = tk.Button(root, text="Forward 2s", command=btn_forward_click)
    btn_backward_2s = tk.Button(root, text="Backward 2s", command=btn_backward_click)
    btn_left_2s = tk.Button(root, text="Left 2s", command=btn_left_click)
    btn_right_2s = tk.Button(root, text="Right 2s", command=btn_right_click)

    velocity_slider.pack()
    canvas.pack()
    btn_forward.pack()
    btn_backward.pack()
    btn_left.pack()
    btn_right.pack()
    btn_stop.pack()
    odom_label.pack()
    btn_forward_2s.pack()
    btn_backward_2s.pack()
    btn_left_2s.pack()
    btn_right_2s.pack()

    ros_thread = threading.Thread(target=ros_spin, args=(node,))
    ros_thread.start()

    root.mainloop()

if __name__ == '__main__':
    main()
