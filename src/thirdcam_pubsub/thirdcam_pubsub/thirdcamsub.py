import serial
import warnings
import serial.tools.list_ports
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    if 'Serial' in p.description
]
if not arduino_ports:
    raise IOError("No Arduino found")
if len(arduino_ports) > 1:
    warnings.warn('Multiple Arduinos found - using the first')

baud_rate=9600
ser = serial.Serial(arduino_ports[0], baud_rate, timeout=1)
print (arduino_ports[0])

time.sleep(2)

def send_key_to_arduino(key):
    ser.write(key.encode())

class KeypressSubscriber(Node):
    def __init__(self):
        super().__init__('thirdcam_sub')
        self.subscription = self.create_subscription(String,'thirdcamcmd',self.listener_callback,10)
        self.subscription
    def listener_callback(self, msg):
        self.get_logger().info('Receiving keypresses "%s"' % msg.data)
        cmdlist = ['w','a','s','d','f','r']
        if msg.data in cmdlist:
            self.get_logger().info('send arduino "%s"' % msg.data)
            send_key_to_arduino(msg.data)
        
def main(args=None):
    rclpy.init(args=args)
    thirdcam_sub = KeypressSubscriber()
    rclpy.spin(thirdcam_sub)

    thirdcam_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    ser.close()
