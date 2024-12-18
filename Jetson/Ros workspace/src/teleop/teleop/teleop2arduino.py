import rclpy
from rclpy.node import Node
import serial
import struct
import time
from std_msgs.msg import String

#Arduino port
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,'tera_teleop',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        info = msg.data.split(",")
        packed = arduino.write(struct.pack("<2h", *[int(info[0]), int(info[1])]))
        print(f"Sent: {packed}")
        time.sleep(0.1)
    
        #Wait until Arduino receives the data and sends it back
        while True:
            try:
                (val, ) = struct.unpack("h", arduino.read(struct.calcsize("h")))
                print(f"Received: {val}")
                break
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
