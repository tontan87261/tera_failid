import folium
import webbrowser
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import threading

class GPSPlotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')
        self.map = None
        self.points = []
        self.refresh_timer = None

    def create_map(self):
        # Initialize the map
        self.map = folium.Map(location=[0, 0], zoom_start=10, tiles=None)

        # Add the Esri Satellite tile layer
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='Esri Satellite',
            overlay=True
        ).add_to(self.map)

        # Add a layer control to toggle the tile layer
        folium.LayerControl().add_to(self.map)

        # Save the map to an HTML file and open it in the browser
        map_file_path = 'gps_map.html'
        self.map.save(map_file_path)
        webbrowser.open(map_file_path, new=0)  # Use new=0 to open in the same browser tab

    def listener_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        self.points.append([latitude, longitude])

    def update_map(self):
        if self.map is not None:
            # Clear the map and redraw it with the new points
            self.map = folium.Map(location=self.points[0], zoom_start=15, tiles=None)

            # Create and add the polyline
            folium.PolyLine(locations=self.points, color='blue').add_to(self.map)

            # Add the Esri Satellite tile layer
            folium.TileLayer(
                tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                attr='Esri',
                name='Esri Satellite',
                overlay=True
            ).add_to(self.map)

            # Add a layer control to toggle the tile layer
            folium.LayerControl().add_to(self.map)

            # Save the map to an HTML file and refresh the browser
            map_file_path = 'gps_map.html'
            self.map.save(map_file_path)
            webbrowser.open(map_file_path, new=0)  # Use new=0 to open in the same browser tab

            # Schedule the next map update
            self.refresh_timer = threading.Timer(5, self.update_map)
            self.refresh_timer.start()

def main():
    rclpy.init()
    gps_plotter = GPSPlotter()

    # Start ROS spinning in a separate thread
    executor_thread = threading.Thread(target=rclpy.spin, args=(gps_plotter,), daemon=True)
    executor_thread.start()

    # Create the map
    gps_plotter.create_map()

    try:
        # Keep the script running to listen for ROS messages
        executor_thread.join()
    except KeyboardInterrupt:
        print("Shutting down GPS Plotter.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()