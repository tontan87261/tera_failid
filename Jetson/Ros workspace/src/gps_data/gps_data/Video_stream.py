from threading import Thread
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')

import cairo
import pymap3d as pm
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
from time import time
from gi.repository import Gst, GLib, GstVideo

class Gstream(Node):
    def __init__(self):
        super().__init__('GPS_subscriber')

        self.hosting_ip = "10.0.3.19"
        self.user_data = {"video_info": None}
        self.speed_text = 0  # Will be updated with calculated speed
        self.offset_x, self.offset_y = 0, 0  # Starting position of the map overlay
        
        # GPS data initialization
        self.lat0, self.lon0, self.h0 = 58.3428685594, 25.5692475361, 91.357
        self.prev_latitude, self.prev_longitude, self.prev_time = None, None, None
        self.path = []  # Store live GPS path
        
        # Load the original CSV data for the blue line
        self.original_csv_path = "/home/tera/ros2_ws2/src/gps_data/gps_data/gps_data_tera_new.csv"
        self.load_original_csv_data(self.original_csv_path)

        # Live data storage path
        self.live_data_path = "/home/tera/ros2_ws2/src/gps_data/gps_data/live_data_storage.csv"

        # ROS 2 subscriber setup
        self.subscription = self.create_subscription(
            String,
            'gps_data_tera',
            self.listener_callback,
            10
        )

    def load_original_csv_data(self, csv_file_path):
        gps_df = pd.read_csv(csv_file_path)
        self.original_path = gps_df[["Latitude", "Longitude"]].values.tolist()
        
        # Debugging: Check loaded original path
        print("Original Path Coordinates:", self.original_path)

    def listener_callback(self, msg):
        info = msg.data.split(',')
        current_latitude = float(info[0])
        current_longitude = float(info[1])
        current_time = time()

        # Debugging: print the current latitude and longitude
        print(f"Received GPS: lat={current_latitude}, lon={current_longitude}")

        if self.prev_latitude is not None and self.prev_longitude is not None:
            distance_m = self.calculate_distance(
                self.prev_latitude, self.prev_longitude,
                current_latitude, current_longitude
            )

            time_diff = current_time - self.prev_time
            if time_diff > 0:
                speed_m_s = distance_m / time_diff
                self.speed_text = round(speed_m_s * 3.6, 2)  # Convert m/s to km/h
            
            # Only update path if movement is greater than 0.5 meters
            if distance_m > 0.5:
                print(f"Current Coordinates: lat={current_latitude}, lon={current_longitude}")  # Debugging statement
                self.path.append((current_latitude, current_longitude))
                
                # Save live GPS coordinates to the CSV
                self.save_live_data_to_csv(current_latitude, current_longitude)

        # Update previous values
        self.prev_latitude = current_latitude
        self.prev_longitude = current_longitude
        self.prev_time = current_time

    def save_live_data_to_csv(self, latitude, longitude):
        new_data = pd.DataFrame({"Latitude": [latitude], "Longitude": [longitude]})
        new_data.to_csv(self.live_data_path, mode='a', header=False, index=False)
        print(f"New GPS data saved: lat={latitude}, lon={longitude}")  # Debugging

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        x1, y1, z1 = pm.geodetic2enu(lat1, lon1, self.h0, self.lat0, self.lon0, self.h0)
        x2, y2, z2 = pm.geodetic2enu(lat2, lon2, self.h0, self.lat0, self.lon0, self.h0)
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def normalize_coordinates(self, lat, lon):
        # Normalize the latitude and longitude for drawing
        normalized_x = (lon - self.lon0) * 10000  # Scale factor can be adjusted
        normalized_y = (lat - self.lat0) * 10000  # Scale factor can be adjusted
        return normalized_x, normalized_y

    def draw_overlay(self, cairo_ctx):
        text = f"Speed: {self.speed_text} km/h"
        cairo_ctx.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cairo_ctx.set_font_size(20)
        cairo_ctx.set_source_rgba(1, 1, 1, 1)  # White text
        cairo_ctx.move_to(575, 700)  # Positioning the speed text
        cairo_ctx.show_text(text)

        # Draw the original path (Blue)
        if self.original_path:
            cairo_ctx.set_source_rgba(0, 0, 1, 1)  # Blue color for original path
            cairo_ctx.set_line_width(2)  # Line width

            normalized_start = self.normalize_coordinates(self.original_path[0][0], self.original_path[0][1])
            cairo_ctx.move_to(*normalized_start)
            for (lat, lon) in self.original_path[1:]:
                normalized_coords = self.normalize_coordinates(lat, lon)
                cairo_ctx.line_to(*normalized_coords)
            cairo_ctx.stroke()

        # Draw the live GPS path as a green line
        if self.path:
            print("Drawing live GPS path...")  # Debugging statement
            cairo_ctx.set_source_rgba(0, 1, 0, 1)  # Green color for live path
            cairo_ctx.set_line_width(2)  # Line width

            # Normalize coordinates for drawing
            normalized_start = self.normalize_coordinates(self.path[0][0], self.path[0][1])
            cairo_ctx.move_to(*normalized_start)
            for (lat, lon) in self.path[1:]:
                normalized_coords = self.normalize_coordinates(lat, lon)
                cairo_ctx.line_to(*normalized_coords)
            cairo_ctx.stroke()

    def on_draw_callback(self, overlay, cairo_ctx, timestamp, duration, user_data):
        if self.user_data["video_info"] is None:
            return
        self.draw_overlay(cairo_ctx)

    def on_caps_callback(self, overlay, caps, user_data):
        video_info = GstVideo.VideoInfo()
        video_info.from_caps(caps)
        self.user_data["video_info"] = video_info

    def start_stream(self):
        Gst.init("")
        self.main_loop = GLib.MainLoop()
        Thread(target=self.main_loop.run).start()

        self.pipeline = Gst.parse_launch(
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=1280,height=720 ! "
            "jpegdec ! "
            "videoconvert ! "
            "cairooverlay name=overlay ! "
            "videoconvert ! "
            "x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! "
            f"rtph264pay ! "
            f"udpsink host={self.hosting_ip} port=5000 sync=false"
        )

        overlay = self.pipeline.get_by_name("overlay")
        overlay.connect("draw", self.on_draw_callback, self.user_data)
        overlay.connect("caps-changed", self.on_caps_callback, self.user_data)

        self.pipeline.set_state(Gst.State.PLAYING)

def main(args=None):
    rclpy.init(args=args)
    gstream_node = Gstream()
    
    # Start the GStreamer pipeline in a separate thread
    Thread(target=gstream_node.start_stream).start()

    rclpy.spin(gstream_node)

    gstream_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
