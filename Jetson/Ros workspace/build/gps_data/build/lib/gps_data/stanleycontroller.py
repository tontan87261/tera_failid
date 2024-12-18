import numpy as np
import wayfinder
from TERA_gps import GpsUtil
from numpy import genfromtxt

k = 0.5
Kp = 1.0
dt = 0.1
L = 1.0
max_steer = np.deg2rad(45)
GPS = GpsUtil()
#rclpy.init(args=None)
run_once = 0

def h_err_corr(next_heading, current_heading): # Calculates the difference between the robot's current heading and the target heading
    return next_heading - current_heading

def xt_err_corr(target_position, current_position, current_heading, k, Kp, k_s): # The cross-track error is the distance between the vehicle and the ideal traiectory
    v_p = np.array(target_position[0]) - np.array(current_position)
    v_t = np.array([np.cos(current_heading), np.sin(current_heading)])
    xt_error = np.linalg.norm(np.cross(v_p, v_t))
    xt_err_correction = np.arctan((xt_error * k) / (Kp + k_s))
    return xt_err_correction

def steering_control(next_heading, current_heading, target_position, current_position, k, v, k_s): # The steering control is the sum of the heading error correction and the cross-track error correction. 
    heading_err = h_err_corr(next_heading, current_heading)
    xt_err = xt_err_corr(target_position, current_position, current_heading, k, v, k_s)

    return heading_err + xt_err 

def get_min_dist(): # This simply calls out a function that returns the closest point on the gps trace in reference to the robot's current location.
    return wayfinder.get_target_location()

def implementation():
    last_x, last_y = 0, 0
    #GPS.get_gps_data()
    #GPS.publish_gps_data()
    current_position = GPS.get_gps_ENU()
    #current_position = [4.1563, -175.085]
    nearest_point = get_min_dist()
    trace_length = len(wayfinder.get_gps_trace())
    data_array = wayfinder.get_gps_trace()
    for i in range(nearest_point[1], (trace_length - 1)):
        #print(f"current position: ", current_position)
        #print(nearest_point[1])
        #print(data_array[i+5, 0])
        #print(data_array[i, 0])
        #print(data_array[i+5, 1])
        #print(data_array[i, 1])
        next_heading = np.arctan2(data_array[i+3, 0] - data_array[i, 0], data_array[i+3, 1] - data_array[i, 1])
        current_heading = np.arctan2(current_position[0] - last_x, current_position[1] - last_y)

        target_position = wayfinder.get_target_location()

        steering_command = np.rad2deg(np.clip(steering_control(next_heading, current_heading, target_position, current_position, k, 1.5, 1), -max_steer, max_steer))

        def map_range(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
        
        steering_command = int(map_range(steering_command, -45, 45, 0, 655))

        #print(steering_command)
        if steering_command == None:
            #print("steering command none")
            return 0
        else:
            #print("steering command")
            return steering_command