#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from find_ports import search_port

import serial
import pynmea2
import socket

GPS_port = search_port("u-blox GNSS receiver")

try:
    ser = serial.Serial(GPS_port, 9600, timeout=1)
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
        super().__init__('gps_publisher')
        self.baudrate = 9600
        self.serial_port = GPS_port # GPS serial port
        
        self.publisher = self.create_publisher(String, 'gps_data_tera', 1)
        timer_period = 0.1  # seconds
        self.timer_raw = self.create_timer(timer_period, self.publish_gps_data)

        if rclpy.ok:
            pass
        else:
            #print("running")
            rclpy.init(args=None)
            super().__init__('gps_publisher')

    def open_serial_connection(self):
        try:
            self.ser_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            #print("Serial open")
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
                    gps_qual = gga_msg.gps_qual
                    num_sats = gga_msg.num_sats

                    return latitude, longitude, altitude, gps_qual, num_sats
            except Exception as e:
                self.get_logger().error(f"Error reading GPS data: {e}")
        return None, None, None, None, None

    def publish_gps_data(self):
        latitude, longitude, altitude, gps_qual, num_sats = self.get_gps_data()
        #print("publishing")
        if latitude is not None and longitude is not None and altitude is not None:
            gps_data_str = f"{latitude}, {longitude}, {altitude}, {gps_qual}, {num_sats}"
            msg = String()
            msg.data = gps_data_str
            self.publisher.publish(msg)
            print(f"Lat: {latitude}, Lon: {longitude}, Alt: {altitude}, Qual: {gps_qual}, Sats: {num_sats}")
            #self.get_logger().info(f"Published GPS data: {gps_data_str}")
            
        else:
            self.get_logger().warning("Failed to retrieve valid GPS data:")
            #'''
            msg = String()
            msg.data = "11.22,11.22,11.22"
            self.publisher.publish(msg)
            #'''

def main():
    rclpy.init(args=None)
    gps_publisher = GpsPublisher()
    #gps_publisher.__init__()
    gps_publisher.open_serial_connection()  # Open serial connection
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()