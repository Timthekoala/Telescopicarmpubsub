# # ROS Client Library for Python
#import rclpy
 
# # Handles the creation of nodes
#from rclpy.node import Node
 
# # Enables usage of the String message type
#from std_msgs.msg import String
 
# class MinimalPublisher(Node):
#   def __init__(self):
#     super().__init__('thirdcam_pub')
     
#     # Create the publisher. This publisher will publish a String message
#     # to the addison topic. The queue size is 10 messages.
#     self.publisher_ = self.create_publisher(String, 'thirdcamcmd', 1)
     
#     # We will publish a message every 0.5 seconds
#     timer_period = 0.1  # seconds
     
#     # Create the timer
#     self.timer = self.create_timer(timer_period, self.timer_callback)
  
#     # Initialize a counter variable
#     self.i = 0
 
#   def timer_callback(self):
#     """
#     Callback function.
#     This function gets called every 0.5 seconds.
#     """
#     # Create a String message
#     msg = String()
 
#     # Set the String message's data
#     msg.data = '  %d' % self.i
     
#     # Publish the message to the topic
#     self.publisher_.publish(msg)
     
#     # Display the message on the console
#     self.get_logger().info('Publishing: "%s"' % msg.data)
 
#     # Increment the counter by 1    
#     self.i += 1
 
# def main(args=None):
 
#   # Initialize the rclpy library
#   rclpy.init(args=args)
 
#   # Create the node
#   minimal_publisher = MinimalPublisher()
 
#   # Spin the node so the callback function is called.
#   rclpy.spin(minimal_publisher)
 
#   # Destroy the node explicitly
#   # (optional - otherwise it will be done automatically
#   # when the garbage collector destroys the node object)
#   minimal_publisher.destroy_node()
 
#   # Shutdown the ROS client library for Python
#   rclpy.shutdown()
 
# if __name__ == '__main__':
#   main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeypressPublisher(Node):
    def __init__(self):
        super().__init__('thirdcam_pub')
        self.publisher_ = self.create_publisher(String, 'thirdcamcmd', 10)
        self.get_logger().info('Publishing keypresses. Press Ctrl+C to stop.')

    def read_keypress(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_keypresses(self):
        while rclpy.ok():
            key = self.read_keypress()
            if key:
                msg = String()
                msg.data = key
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {key}')

def main(args=None):
    rclpy.init(args=args)
    node = KeypressPublisher()
    
    try:
        node.publish_keypresses()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()