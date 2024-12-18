import rclpy
from std_msgs.msg import String
import pymap3d as pm
from rclpy.node import Node
import pandas as pd
import matplotlib.pyplot as plt

#Salvestatud raja lugemine
gps_df = pd.read_csv("~/Downloads/tera_path.csv")
#print(gps_df)
x = gps_df["Latitude"].values
y = gps_df["Longitude"].values
latitude, longitude = 0,0
plt.figure()
#Raja kaardil kuvamine
plt.plot(x, y, 'b-', label="Path")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("TERA")

#GPS Subscriber
class GPS_Subscriber(Node):
    def __init__(self):
        super().__init__('Stanley_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps_data_tera',
            self.listener_callback,
            10)
        self.subscription
        self.lat0, self.lon0, self.h0 = 58.3428685594, 25.5692475361, 91.357
        self.latitude, self.longitude, self.altitude = 0, 0, 0
        self.ENU_x, self.ENU_y = 0, 0
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard "%s"' %msg.data)
        info = msg.data.split(',')
        self.latitude = float(info[0])
        self.longitude = float(info[1])
        self.altitude = float(info[2])
        #Koordinaatide ENUks tegemine
        self.ENU_x, self.ENU_y, ENU_z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.lat0, self.lon0, self.h0)
        #Hetkeasukoha kaardil kuvamine
        plt.plot(self.ENU_x, self.ENU_y,'r+')
        plt.pause(0.1)

    def get_coords(self):
        return self.latitude, self.longitude

if __name__ == '__main__':
    rclpy.init(args=None)
    gps = GPS_Subscriber()
    rclpy.spin(gps)
    #plt.show()
   