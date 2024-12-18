#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import pynmea2
import csv
from gmplot import GoogleMapPlotter
import socket

csv_file_path = 'gps_data_tera.csv'  # Path to the CSV file

try:
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    s = socket.socket()
    port = 8002
    s.connect(('213.168.5.170', port))
except serial.SerialException as e:
    print("Error: %s", e)

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher = self.create_publisher(String, 'gps_data_tera', 1)
        timer_period = 0.1  # seconds
        self.timer_raw = self.create_timer(timer_period, self.publish_gps_data)
        self.serial_port = '/dev/ttyACM1'  # GPS serial port
        self.baudrate = 9600
        self.serial_conn = None

        self.csv_file = open(csv_file_path, 'w')  # Open CSV file for writing
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Latitude', 'Longitude', 'Altitude'])  # Write header


    def open_serial_connection(self):
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print("Serial open")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

    def get_gps_data(self):
        if self.serial_conn is not None and self.serial_conn.is_open:
            try:
                line = self.serial_conn.readline().decode('utf-8')
                print(line)
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
            gps_data_str = f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}"
            msg = String()
            msg.data = gps_data_str
            self.publisher.publish(msg)
            self.get_logger().info(f"Published GPS data: {gps_data_str}")
            
            # Write data to CSV file
            self.csv_writer.writerow([latitude, longitude, altitude])
            self.csv_file.flush()  # Ensure data is written to the file immediately
        else:
            self.get_logger().warning("Failed to retrieve valid GPS data:")


def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GpsPublisher()
    gps_publisher.open_serial_connection()  # Open serial connection
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except:
        csv_file_path.close()
