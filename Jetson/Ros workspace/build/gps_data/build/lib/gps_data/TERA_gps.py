#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import pynmea2
import socket

try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    s = socket.socket()
    port = 8002
    s.connect(('213.168.5.170', port))
except serial.SerialException as e:
    print("Error: %s", e)

global latitude
global longitude
global altitude

latitude, longitude, altitude = 0, 0, 0

class GpsPublisher(Node):
    def __init__(self):
        rclpy.init(args=None)

        super().__init__('gps_publisher')
        self.baudrate = 9600
        self.serial_port = '/dev/ttyACM1'  # GPS serial port
        
        self.publisher = self.create_publisher(String, 'gps_data_tera', 1)
        timer_period = 0.1  # seconds
        self.timer_raw = self.create_timer(timer_period, self.publish_gps_data)

        if rclpy.ok:
            pass
        else:
            print("running")
            rclpy.init(args=None)
            super().__init__('gps_publisher')

    def open_serial_connection(self):
        try:
            self.ser_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print("Serial open")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
        return self.ser_conn

    def get_gps_data(self):
        self.ser_conn = self.open_serial_connection()
        if self.ser_conn is not None and self.ser_conn.is_open:
            try:
                line = self.ser_conn.readline().decode('utf-8')
                if line.startswith('$GNGGA'):
                    gga_msg = pynmea2.parse(line)
                    latitude = gga_msg.latitude
                    longitude = gga_msg.longitude
                    altitude = gga_msg.altitude

                    return latitude, longitude, altitude
            except Exception as e:
                self.get_logger().error(f"Error reading GPS data: {e}")
        return None, None, None

    def publish_gps_data(self):
        latitude, longitude, altitude = self.get_gps_data()
        if latitude is not None and longitude is not None and altitude is not None:
            gps_data_str = f"{latitude}, {longitude}, {altitude}"
            msg = String()
            msg.data = gps_data_str
            self.publisher.publish(msg)
            self.get_logger().info(f"Published GPS data: {gps_data_str}")
            
        else:
            self.get_logger().warning("Failed to retrieve valid GPS data:")
            

def main():
    
    gps_publisher = GpsPublisher()
    #gps_publisher.__init__()
    gps_publisher.open_serial_connection()  # Open serial connection
    rclpy.spin(gps_publisher.publish_gps_data())
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()