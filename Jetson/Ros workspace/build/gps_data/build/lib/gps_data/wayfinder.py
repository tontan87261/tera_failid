from TERA_gps import GpsUtil
import math
import pandas as pd
from numpy import genfromtxt
import numpy as np
import rclpy



data = pd.read_csv(r'/home/tera/TERA/NEW/TERA/Jetson/GPS_utils/gps_ENU.csv', header=0, usecols=['X', 'Y'])

gps = GpsUtil()
#rclpy.init(args=None)
run_once = 0

def get_target_location(self): # Acquiring the closest or next point to which to move
    global target_location
    i = 0
    last_min_dist = 100000050505050
    target_location = []
    if target_location == []: # If there is no target location, the program finds the closest point to the robot.
        for i in data.index:
            _,_,_,_,current_x, current_y,_ = gps.get_gps_data()
            #current_x = 4.1563
            #current_y = -175.085
            check_x = data.at[i, 'X']
            check_y = data.at[i, 'Y']
            check_location = [check_x, check_y]
            current_min_dist = math.dist([check_x, check_y], [current_x, current_y]) # Iterating through every fifth points
            
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

def get_gps_trace(): # This simply returns the gps trace array.
    return np.array(data)