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

#Global variables:
max_steer = 40                              #Maximum steering value in degrees
gap = 0.4                                   #The minimum gap between two path points
yaws = []                                   #Array of directions of the vectors between path points
vector_lengths = []                         #Array of length of the vectors between path points
prev_time = 0                               #Time of previous loop
#prev_loc_x = 0                              #x coordinate of previous loop
#prev_loc_y = 0                              #y coordinate of previous loop
#delta = 0                                   #current steering angle
#delta_prev = 0                              #previois steering angle


#Find the port connected to arduino and connect to it
arduino = serial.Serial(port=search_port("Arduino"), baudrate=115200, timeout=0.1)

#Read the saved path file
with open("/home/tera/ros2_ws2/src/gps_data/gps_data/gps_path1.csv", "r") as csvfile:
    reader = csv.reader(csvfile, quotechar='|')
    data = []
    next(reader, None)
    for row in reader:
        #print(repr(row))
        x = float(row[0])
        y = float(row[1])
        z = float(row[2])
        data.append([x, y])
    print(f"Path length is {len(data)} points")
    #Save x and y coordinates in different arrays for easier use 
    x_gapped = [point[0] for point in data]
    y_gapped = [point[1] for point in data]
    
#GPS subscriber
class GPS_Subscriber(Node):
    #Initialising GPS data publisher
    def __init__(self):
        super().__init__('Stanley_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gps_data_tera',
            self.listener_callback,
            10)
        self.subscription
        #Coordinates of GPS base station
        self.lat0, self.lon0, self.h0 = 58.3428685594, 25.5692475361, 91.357
        #Robot coordinate variables
        self.latitude, self.longitude, self.altitude = 0, 0, 0
        #Robot ENU coordinate variables
        self.ENU_x, self.ENU_y = 0, 0
        
        
    #Callback function that is called when receiving data from GPS
    def listener_callback(self, msg):
        self.get_logger().info('I heard "%s"' %msg.data)
        info = msg.data.split(',')
        self.latitude = float(info[0])
        self.longitude = float(info[1])
        self.altitude = float(info[2])
        self.ENU_x, self.ENU_y, ENU_z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.lat0, self.lon0, self.h0)
    
    #Function to return ENU coordinates outside of the class
    #ENU = East, north, up local tangent plane coordinates
    def get_ENU(self):
        return self.ENU_x, self.ENU_y
    
#Robot command publisher
class CommandPublisher(Node):
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

class AdvancedStanley():
    def __init__(self):
        self.delta = 0
        self.delta_prev = 0
        self.prev_loc_y = 0                                              #hotfix for missing local variable
        self.prev_loc_x = 0
    #Calculate the lengths and directions of vectors between path points
    def path_vectors():
        l = 0
        i = 0
        #Vector length calculation
        while i < (len(x_gapped) - 1):
            l = np.sqrt(np.square(x_gapped[i + 1] - x_gapped[i]) + np.square(y_gapped[i +1] - y_gapped[i]))
            if l < gap:             
                del x_gapped[i + 1]
                del y_gapped[i + 1]
            else:
                l = 0
                vector_lengths.append(l)
                i+=1
        #Calculate direction of the vector
        for i in range (len(x_gapped) - 1):
            d_x = x_gapped[i + 1] - x_gapped[i]
            d_y = y_gapped[i + 1] - y_gapped[i]
            yaw = np.arctan2(d_y, d_x)
            yaws.append(yaw)
        #print(f"Length of yaws is {len(yaws)}, and vector lengths is {len(vector_lengths)}")
        return np.array(yaws), np.array(vector_lengths)

    #Find the closest point on the path to asked point
    def get_target_location(x_arr, y_arr, x, y):
            i = 0
            min_dist = 100000000
            for i in range(len(x_arr)):
                current_min_dist = math.dist([x_arr[i], y_arr[i]], [x, y])

                if current_min_dist < min_dist:
                    min_dist = current_min_dist
                    min_dist_idx = i
            #returns closest point and previous closest point
            return [data[min_dist_idx], data[min_dist_idx - 1], min_dist_idx]
        
    #Calculate curvature of each path point
    def calculate_curvature():
        #Source: https://stackoverflow.com/questions/28269379/curve-curvature-in-numpy/28270382#28270382
        #compute the derivatives of each variable and put them together
        dx_dt = np.gradient(x_gapped)
        dy_dt = np.gradient(y_gapped)
        d2x_dt2 = np.gradient(dx_dt)
        d2y_dt2 = np.gradient(dy_dt)
        curvature = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5
        return curvature

    #Calculate robot speed and direction
    def robot_speed(self,x,y,prev_time):                                                   #input current robot coordinates
        curr_time = time.time()
        delta_t = curr_time - prev_time                                     #calculate time delta between current and previous loops 
        distance = np.sqrt(x - self.prev_loc_x) + np.square(y - self.prev_loc_y)      #distance between this and last point
        direction = np.arctan2(y - self.prev_loc_y, x - self.prev_loc_x)
        velocity = distance / delta_t                                       #robot speed (m/s)
        self.prev_loc_x = x#update previous x and y coords
        self.prev_loc_y = y#update previous x and y coords
        prev_time = curr_time                                               #update previous time value
        return velocity, delta_t, direction
        
    #Calculate curvature of the path after t_ff seconds
    def kappa_feedforward(self,x, y, v, psi, t_ff, kappa, x_gapped, y_gapped):                       #Inputs: x and y coordinates of robot, robot speed, robot directon, feedforward time
        l = v * t_ff                                                        #calculate the distance travelled in t_ff time 
        x_ff = x + l * math.cos(psi)                                        #calculate x coord of robot after t_ff time
        y_ff = y + l * math.sin(psi)                                        #calculate y coord of robot after t_ff time#calculate x coord of robot after t_ff time
        ff_target = self.get_target_location(x_gapped, y_gapped, x_ff, y_ff)
        return kappa[ff_target[2]]

    def advanced_stanley(self):
        #Inputs:
        gps = GPS_Subscriber()                                              # init gps subscriber class
        x_gps, y_gps = gps.get_ENU()                                        #Reference x and y coord (Position of gps)                            
        v_r, delta_time, psi_r = self.robot_speed(x_gps, y_gps, prev_time)                                     #v_r = robot speed (m/s), time since last iteration (Seconds), direction of robot
        kappa = self.calculate_curvature()                                       #Curves of each path point (array)
        path_dirs, path_lens = self.path_vectors()
        nearest_point_r = self.get_target_location(x_gapped, y_gapped, x_gps, y_gps)
        kappa_ref = kappa[nearest_point_r[2]]                               #curvature of path at the nearest path point
        psi_r_ref = path_dirs[nearest_point_r[2] - 1]                           #direction of the path at the nearest path point
        x_r_ref = nearest_point_r[0][0]                                        #x coord of closest point of path to rear axle
        y_r_ref = nearest_point_r[0][1]                                        #y coord of closest point of path to rear axle
        
        delta_yaw = self.delta - self.delta_prev                                      # compute yaw difference between samples
        psi_dot_r = delta_yaw / delta_time                                  # compute yaw rate    


        #Vehicle Parameters

        a                          =  0.39          # distance from gps to front axle in m
        b                          =  0.61          # distance from gps to rear axle in m   
        m                          =  40            # mass of vehicle in kg 
        c_y_f                      =  28000         # tire stiffness of the tire pair at the front axle in N/rad
        c_y_r                      =  26000         # tire stiffness of the tire pair at the rear axle in N/rad

        #Stanley Controller Settings

        t_ff              =  0.18                   # feedforward time t_ff in s   
        k                 =  3                      # k
        k_soft            =  1                      # k_soft
        k_d_yaw           =  0.125                  # k_d_yaw  
        k_d_steer         =  0                      # k_d_steer 

        l = a + b                                   # wheelbase

        k_ag_f = m / (c_y_f * (1 + a / b))          # slip angle coefficient front
        k_ag_r = m / (c_y_r * (1 + b / a))          # slip angle coefficient rear
        #---------------------------------------------------------------------------------------------------------

        psi_dot_kappa_ref = v_r * kappa_ref                     # expected yaw rate (velocity of robot * curvature of the path)

        theta_ss_f = k_ag_f * abs(v_r) * psi_dot_kappa_ref      # steady state yaw angle front
        theta_ss_r = k_ag_r * abs(v_r) * psi_dot_kappa_ref

        #kappa_ref_ff = curvature of the path after t_ff seconds
        kappa_ref_ff = self.kappa_feedforward(x_gps, y_gps, v_r, psi_r, t_ff, kappa, x_gapped, y_gapped)

        sign_v_r = math.copysign(1, v_r)                        # Velocity of rear axle, + is forward, - is reverse

        #x and y coordinate rear
        x_r = x_gps - b * math.cos(psi_r)
        y_r = x_gps - b * math.cos(psi_r)

        #x and y coordinate front
        x_f = x_gps + a * math.cos(psi_r)
        y_f = y_gps + a * math.sin(psi_r) 

        #Crosstrack error rear
        e_lat_r = math.cos(psi_r_ref) * (y_r_ref - y_r) - math.sin(psi_r_ref) * (x_r_ref - x_r)

        # x and y coordinate reference, see equation (4)
        x_f_ref = x_r_ref + l * math.cos(psi_r_ref + theta_ss_r)
        y_f_ref = y_r_ref + l * math.sin(psi_r_ref + theta_ss_r)

        #Curvature based steering angle
        delta_kappa_ref = math.atan((kappa_ref * l - math.sin(theta_ss_r)) / math.cos(theta_ss_r))

        #Reference orientation of front wheels
        psi_front_ref = psi_r_ref + theta_ss_r + delta_kappa_ref

        #Crosstrack error front
        e_lat_f = math.cos(psi_front_ref) * (y_f_ref - y_f) - math.sin(psi_front_ref) * (x_f_ref - x_f)

        #Lateral error front/rear based on driving direction
        e_lat = 0.5 * e_lat_f * (1 + sign_v_r) + 0.5 * e_lat_r * (1 - sign_v_r)

        #Curvature based on feedforward term in Enhanced Stanley control law 
        delta_kappa_ff = math.atan((kappa_ref_ff * l - math.sin(theta_ss_r)) / math.cos(theta_ss_r))

        #actual orientation error:
        theta_r = ((psi_r_ref - psi_r + math.pi % 2*math.pi) - math.pi) + theta_ss_r        # % operator returns the remainder after division, 
        #                                                                                   #e.g. 23 / 5 = 3 (because 20 is the biggest nr close to 23 that is dividable by 5)

        #Additional terms in Stanley control law

        delta_add = k_d_yaw * (psi_dot_kappa_ref - psi_dot_r) * sign_v_r + k_d_steer * (self.delta_prev - self.delta) + theta_ss_f        

        #Final Calculation of Enhanced Stanley:

        delta_stanley_enhanced = delta_kappa_ff + theta_r * sign_v_r + math.atan(k*e_lat/(k_soft+abs(v_r))) + delta_add
        self.delta = delta_stanley_enhanced          #Update current steering value
        if self.delta_prev != self.delta:                 #Update previous steering value (omalooming)
            self.delta_prev = self.delta

        #--------------------------------------------DEBUG PRINTING--------------------------------------------------------------------------
        psi_r_ref = path_dirs[nearest_point_r[2] - 1]                           #direction of the path at the nearest path point
        x_r_ref = nearest_point_r[0][0]                                        #x coord of closest point of path to rear axle
        y_r_ref = nearest_point_r[0][1]                                        #y coord of closest point of path to rear axle
        print(f"Robot speed: {v_r}\nTime since last iteration (Seconds): {delta_time}")
        print(f"Direction of robot: {psi_r}\nNearest point value at rear wheels: {nearest_point_r[0]}")
        print(f"Nearest point idx at rear wheels: {nearest_point_r[2]}\nCurvature of path at the nearest path point {kappa_ref}")
        print(f"Direction of the path at the nearest path point (Rear axle): {psi_r_ref}")
        print(f"Closest path point to rear axle: ({x_r_ref}, {y_r_ref})")
        return delta_stanley_enhanced

if __name__ == '__main__':
    rclpy.init(args=None)
    gps = GPS_Subscriber()
    pub = CommandPublisher()
    stan = AdvancedStanley()
    while True:
        rclpy.spin_once(gps)
        #Calculate steering
        steering = stan.advanced_stanley()
        steering_deg = np.rad2deg(steering)
        steering_clipped = np.clip(steering_deg, -max_steer, max_steer)
        print(f"Steeing value in degrees: {steering_deg}")
        print(f"Steering value clipped: {steering_clipped}")
        #Publish robot commands
        pub.input([steering_clipped, 120])
        time.sleep(0.2)