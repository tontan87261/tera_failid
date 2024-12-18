from find_ports import search_port
import math
import numpy as np
import rclpy
import serial
import pandas as pd
import time
from std_msgs.msg import String
import pymap3d as pm
import csv
from rclpy.node import Node
from std_msgs.msg import String

global latitude
global longitude

latitude, longitude, altitude = 0, 0, 0
ENU_x, ENU_y, ENU_z = 0, 0, 0

arduino = serial.Serial(port=search_port("Arduino"), baudrate=115200, timeout=0.1)

with open("/home/tera/ros2_ws2/src/gps_data/gps_data/gps_path1.csv", "r") as csvfile:
    reader = csv.reader(csvfile, quotechar='|')
    next(reader)
    data = []
    for row in reader:
        x = float(row[0])
        y = float(row[1])
        z = float(row[2])
        data.append([x, y])
    print(f"Path length is {len(data)} points")

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'tera_teleop', 10)
        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.input)
        print("Starting Stanley teleop")

    def input(self, data):
        msg = String()
        msg.data = (f"{data[0]},{data[1]}")
        self.publisher_.publish(msg)    
        print(f"Msg: ", msg)

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
        self.ENU_x, self.ENU_y, ENU_z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.lat0, self.lon0, self.h0)


    def get_coordinates(self):
        return self.latitude, self.longitude
        
    
    def get_ENU(self):
        return self.ENU_x, self.ENU_y

class StanleyController():
    def __init__(self):
        self.target_location = []
        self.target_x = 0
        self.target_y = 0
        self.last_x = 0
        self.last_y = 0
        self.xt_err_corr
        self.k = 0.4                        #0.4
        self.Kp = 1.0                       #1.0
        self.v = 1.5                        #1.5
        self.k_s = 1.5                        #1
        #self.dt = 0.1                       #0.1
        #self.L = 1.0                        #1.0
        self.max_steer = np.deg2rad(40)
        self.steering_command = 0
        self.suber = GPS_Subscriber()
        self.nearest_point_idx = 0

        #Simulatsioonis: 
        #k = 1
        #v = 2.5
        #k_s = 0.5

    def get_gps_trace(self):
        return data

    def path_yaws(self, x, y):
        yaws = []

        for i in range(len(x) - 1):
            d_x = x[i+1] - x[i]
            d_y = y[i+1] - y[i]
            yaw = np.arctan2(d_y, d_x)
            yaws.append(yaw)
        yaws.append(yaws[-1])
        #print(len(yaws))
        return np.array(yaws)
    
    #Finding the closest track point to the robot
    def get_target_location(self):
        i = 0
        min_dist = 100000000
        for i in range(len(data)):
            current_min_dist = math.dist(data[i], [self.ENU_x, self.ENU_y])

            if current_min_dist < min_dist:
                min_dist = current_min_dist
                min_dist_idx = i

        return [data[min_dist_idx], data[min_dist_idx - 1], min_dist_idx]

    def xt_err_corr(self, current_heading, k_s, dx, dy): # The cross-track error is the distance between the vehicle and the ideal traiectory

        v_p = np.array([np.sin(current_heading), -np.cos(current_heading)])
        v_t = np.array([dx, dy])
        #print(f"v_t :", v_t)
        xt_error = np.linalg.norm(np.cross(v_p, v_t))
        xt_err_correction = np.arctan((xt_error * self.k) / (self.Kp + k_s))
        return xt_err_correction 

    def steering_control(self, next_heading, current_heading, target_position, current_position, k, v, k_s, path_vector): # The steering control is the sum of the heading error correction and the cross-track error correction. 
        heading_err = np.arctan2(np.sin(path_vector - current_heading), np.cos(path_vector - current_heading))
        dx = (current_position[0] + 0.38 * np.cos(current_heading)) - target_position[0]
        dy = (current_position[1] + 0.38 * np.sin(current_heading)) - target_position[1]
        xt_err = self.xt_err_corr(current_heading, k_s, dx, dy)
        return heading_err + xt_err

    def implementation(self):
        gps_df = pd.read_csv("/home/tera/ros2_ws2/src/gps_data/gps_data/gps_path1.csv")
        current_position = [self.ENU_x, self.ENU_y]
        nearest_point = self.get_target_location()[0]
        self.nearest_point_idx = self.get_target_location()[2]
        next_heading_x = nearest_point[0] - current_position[0]
        next_heading_y = nearest_point[1] - current_position[1]
        next_heading = -np.arctan2(next_heading_y, next_heading_x)
        current_heading = np.arctan2(current_position[1] - self.last_y, current_position[0] - self.last_x)
        x = gps_df["Latitude"].values
        y = gps_df["Longitude"].values
        path_yaw = self.path_yaws(x, y)
        print(f"Path yaw len: {len(path_yaw)}")
        next_path_heading = path_yaw[self.nearest_point_idx]
        self.steering_command = np.rad2deg(np.clip(self.steering_control(next_heading, current_heading, nearest_point, current_position, self.k, self.v, self.k_s, next_path_heading), -self.max_steer, self.max_steer))
        self.last_x = current_position[0]
        self.last_y = current_position[1]
        

        def map_range(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
        
        self.adj_steering_command = int(map_range(self.steering_command, 40, -40, 0, 660))

        if self.adj_steering_command == None:
            return 0
        else:
            return int(self.adj_steering_command)
            
    def send2arduino(self, lat, lon, x, y):
        self.latitude = lat
        self.longitude = lon
        self.ENU_x = x
        self.ENU_y = y
        steering = self.implementation()
        if self.nearest_point_idx == len(data):
            pub.input([330, 0])
            print("----------------------End of path----------------------")
        else:
            pub.input([steering, 120])
        
        time.sleep(0.2)
        '''
        sent = arduino.write(struct.pack("<2h", *[int(steering), int(100)]))
        print("To arduino: ", sent)
        time.sleep(0.5)
        try:
            (val, ) = struct.unpack("h", arduino.read(struct.calcsize("h")))
            print("Received: ", val)
        except:
            print("Got nothing from arduino")
            pass
        '''
        

if __name__ == '__main__':
    rclpy.init(args=None)
    gps = GPS_Subscriber()
    pub =MinimalPublisher()
    stan = StanleyController()
    while True:
        rclpy.spin_once(gps)
        latitude, longitude = gps.get_coordinates()
        x, y = gps.get_ENU()

        stan.send2arduino(latitude, longitude, x, y)