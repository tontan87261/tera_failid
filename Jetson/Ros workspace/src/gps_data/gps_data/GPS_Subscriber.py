import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pymap3d as pm
import csv

csv_file_path = '/home/tera/ros2_ws2/src/gps_data/gps_data/gps_path1.csv'

class GPS_Subscriber(Node):
    def __init__(self):
        #rclpy.init(args=None)
        super().__init__('GPS_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps_data_tera',
            self.listener_callback,
            10)
        self.subscription
        self.lat0, self.lon0, self.h0 = 58.3428685594, 25.5692475361, 91.357
        self.latitude, self.longitude, self.altitude = 0, 0, 0
        self.x, self.y, self.z = 0, 0, 0

        self.csv_file = open(csv_file_path, 'w')  # Open CSV file for writing
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Latitude', 'Longitude', 'Altitude'])

    def listener_callback(self, msg):
        self.get_logger().info('I heard "%s"' %msg.data)
        info = msg.data.split(',')
        self.latitude = float(info[0])
        self.longitude = float(info[1])
        self.altitude = float(info[2])
        self.x, self.y, self.z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.lat0, self.lon0, self.h0)
        self.csv_writer.writerow([self.x, self.y, self.z])
        self.csv_file.flush()

    def get_gps_data(self):
        return self.latitude, self.longitude

    def get_gps_ENU(self):
        return self.x, self.y, self.z

def main(args=None):
    rclpy.init(args=args)

    gps = GPS_Subscriber()

    rclpy.spin(gps)

    gps.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()