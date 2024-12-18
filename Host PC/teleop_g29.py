import rclpy
from rclpy.node import Node
import evdev
from std_msgs.msg import String

#Logitech G29 rool
device = evdev.InputDevice('/dev/input/event26')

#Väärtuste initsialiseerimine
info = [33000, 255, 255]
moving = 0

#Publisheri loomine
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'tera_teleop', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.input)
        print("Starting teleop controls")

    def input(self):
        msg = String()
        #Iga kord kui roolilt info tuleb:
        for event in device.read_loop():

            if event.type == evdev.ecodes.EV_ABS:
                #Rool
                if event.code in [evdev.ecodes.ABS_X]:
                    #print(f"Axis {evdev.ecodes.ABS[event.code][4:]}: {event.value}")
                    info[0] = round(event.value / 100)
                    
                    print(f"Steering: {info[0]}")
                #Gaasipedaal
                if event.code in [evdev.ecodes.ABS_Z]:
                    #print(f"Axis {evdev.ecodes.ABS[event.code][4:]}: {event.value}")
                    info[1] = event.value
                    #print(f"Throttle: {data[1]}")
                    
                #Piduripedaal
                if event.code in [evdev.ecodes.ABS_RZ]:
                    #print(f"Axis {evdev.ecodes.ABS[event.code][4:]}: {event.value}")
                    info[2] = event.value
                    #print(f"Brake: {data[2]}")
            moving = info[2] - info[1]
            msg.data = (f"{info[0]},{moving}")
            self.publisher_.publish(msg)
            print(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
