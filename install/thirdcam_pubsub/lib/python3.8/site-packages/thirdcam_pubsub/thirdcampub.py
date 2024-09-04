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