#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr


class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')
        self.command_publisher = self.create_publisher(String, 'voice_command', 10)
        self.timer = self.create_timer(0.1, self.listen_callback)
        self.recognizer = sr.Recognizer()
        self.get_logger().info('Speaker Node initialized')
        
    def listen_callback(self):
        
        self.timer.cancel()
        
        with sr.Microphone() as source:
            self.get_logger().info("Listening for commands...")
            try:
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.recognizer.listen(source)
                text = self.recognizer.recognize_vosk(audio)
                self.get_logger().info(f"Recognized: {text}")
                
                msg = String()
                msg.data = text.lower()
                self.command_publisher.publish(msg)
                
            except sr.UnknownValueError:
                self.get_logger().info("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results; {e}")
            
            # Reset the timer
            self.timer = self.create_timer(0.1, self.listen_callback)

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()