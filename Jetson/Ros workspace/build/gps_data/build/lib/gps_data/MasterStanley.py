from GPS_Subscriber import GPS_Subscriber
import TERA_gps
import math
import pandas as pd
import numpy as np
import rclpy
import serial
import struct
import time
from std_msgs.msg import String
import pymap3d as pm
from numpy import genfromtxt

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

data = pd.read_csv(r'/home/tera/TERA/NEW/TERA/Jetson/GPS_utils/gps_ENU.csv', header=0, usecols=['X', 'Y'])

class StanleyController():
    def __init__(self):
        self.target_location = []
        self.target_x = 0
        self.target_y = 0
        self.h_err_corr
        self.xt_err_corr
        self.k = 0.5
        self.Kp = 1.0
        self.dt = 0.1
        self.L = 1.0
        self.max_steer = np.deg2rad(45)
        self.steering_command = 0
        self.suber = GPS_Subscriber()
        
    def get_target_location(self):
        i = 0
        last_min_dist = 100000000
        if self.target_location == []:
            for i in data.index:
                current_x, current_y = self.suber.get_gps_data()
                check_x = data.at[i, 'X']
            check_y = data.at[i, 'Y']
            check_location = [check_x, check_y]
            current_min_dist = math.dist([check_x, check_y], [current_x, current_y])

            if (i + 5) not in range(0, len(data)):
                return [[data.at[i, 'X'], data.at[i, 'Y']], i]
            
            else:
                i += 5

            if target_location == None or current_min_dist < last_min_dist: # Assigning closest point
                target_location = check_location
            last_min_dist = current_min_dist
        else:
            target_location = data.at[i+5] # If there is a current closest point, the next one is the fifth one after the current one.
            i += 5
        return [target_location, i]
    
    def h_err_corr(next_heading, current_heading): # Calculates the difference between the robot's current heading and the target heading
        return next_heading - current_heading

    def xt_err_corr(self, target_position, current_position, current_heading, k_s): # The cross-track error is the distance between the vehicle and the ideal traiectory
        v_p = np.array(target_position[0]) - np.array(current_position)
        v_t = np.array([np.cos(current_heading), np.sin(current_heading)])
        xt_error = np.linalg.norm(np.cross(v_p, v_t))
        xt_err_correction = np.arctan((xt_error * self.k) / (self.Kp + k_s))
        return xt_err_correction

    def steering_control(self, next_heading, current_heading, target_position, current_position, k, v, k_s): # The steering control is the sum of the heading error correction and the cross-track error correction. 
        heading_err = self.h_err_corr(next_heading, current_heading)
        xt_err = self.xt_err_corr(target_position, current_position, current_heading, k, v, k_s)
        return heading_err + xt_err
    
    def get_gps_trace(self):
        data_array = genfromtxt(data, delimiter=',')
        data_array = np.round(data_array, 3)

        return data_array


    def implementation(self):
        last_x, last_y = 0, 0
        current_position = self.suber.get_gps_ENU()
        nearest_point = self.get_target_location()
        trace_length = len(self.get_gps_trace())
        data_array = self.get_gps_trace()
        for i in range(nearest_point[1], (trace_length - 1)):
            next_heading = np.arctan2(data_array[i+3, 0] - data_array[i, 0], data_array[i+3, 1] - data_array[i, 1])
            current_heading = np.arctan2(current_position[0] - last_x, current_position[1] - last_y)

            target_position = self.get_target_location()

            self.steering_command = np.rad2deg(np.clip(self.steering_control(next_heading, current_heading, target_position, current_position, self.k, 1.5, 1), -self.max_steer, self.max_steer))

            def map_range(x, in_min, in_max, out_min, out_max):
                return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
            
            self.steering_command = int(map_range(self.steering_command, -45, 45, 0, 655))

            if self.steering_command == None:
                return 0
            else:
                return self.steering_command
            
    def listener_callback(self):
        steering = self.implementation()
        print(type(steering))
        print(f"steering value: ", steering)
        arduino.write(struct.pack("2h", steering, int(100)))
        time.sleep(0.05)
        try:
            (val, ) = struct.unpack("h", arduino.read(struct.calcsize("h")))
        except:
            pass

if __name__ == '__main__':
    #TERA_gps.main()
    stan = StanleyController()
    while True:
        stan.listener_callback()