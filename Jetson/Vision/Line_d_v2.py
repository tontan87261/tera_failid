import cv2
import numpy as np

# IP address of the receiving device (replace with your desired IP)
DEST_IP = "10.0.6.81"  # Change this to the specific IP address
DEST_PORT = 5007        # Port for streaming

# GStreamer pipeline to send video feed
gst_pipeline = (
    f"appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! "
    f"rtph264pay ! udpsink host={DEST_IP} port={DEST_PORT}"
)

# Initialize the VideoWriter for streaming
out_stream = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, 20, (640, 480), True)

# Open the camera
cap = cv2.VideoCapture(0
)  # Adjust camera index if necessary

if cap.isOpened():
    print("Camera opened successfully")
else:
    print("Camera failed to open")
    exit()

max_distance = 300  # Adjust this to your expected max distance in pixels

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    resized_frame = cv2.resize(frame, (640, 480))
    height, width = resized_frame.shape[:2]

    # Perspective transformation
    src_points = np.float32([ 
        [0, height // 2],               # Top-left point of the bottom half
        [width, height // 2],           # Top-right point of the bottom half
        [width, height],                # Bottom-right corner
        [0, height]                     # Bottom-left of the output
    ])
    dst_points = np.float32([
        [0, 0],                         # Map to top-left of the output
        [width, 0],                     # Map to top-right of the output
        [width, height],                # Bottom-right of the output
        [0, height]                     # Bottom-left of the output
    ])

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    birdseye_view = cv2.warpPerspective(resized_frame, matrix, (width, height))

    # Convert the bird's-eye view to HSV color space
    hsv_birdseye = cv2.cvtColor(birdseye_view, cv2.COLOR_BGR2HSV)

    # Define extended HSV ranges for yellow colors
    yellow_lower = np.array([10, 36, 199])  # Adjusted lower bound for broader detection
    yellow_upper = np.array([32, 171, 255])

    # Create mask for yellow spectrum
    yellow_mask = cv2.inRange(hsv_birdseye, yellow_lower, yellow_upper)

    # Dilate the mask to emphasize thin lines
    kernel = np.ones((7, 7), np.uint8)
    dilated_yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=2)

    # Find contours to get the yellow line
    contours, _ = cv2.findContours(dilated_yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if contours are found
    if contours:
        # Sort contours by area and get the largest one (likely the yellow line)
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the centroid of the largest contour (center of the yellow line)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # x coordinate of the center of the yellow line
            cy = int(M["m01"] / M["m00"])  # y coordinate of the center of the yellow line

            # The left dot is always in the center of the frame (middle of the screen)
            left_dot_x = width // 2
            left_dot_y = height // 2

            # Draw the first blue dot (fixed at the center of the screen)
            cv2.circle(birdseye_view, (left_dot_x, left_dot_y), 5, (255, 0, 0), -1)

            # The second dot will be 100 pixels to the left of the first dot
            second_dot_x = left_dot_x - 100
            if second_dot_x >= 0:  # Ensure it's within the frame
                # The second dot will be placed at the y-coordinate of the yellow line at second_dot_x
                # Traverse down the mask to find the y-coordinate where the yellow line is present at second_dot_x
                for i in range(height):
                    if yellow_mask[i, second_dot_x] > 0:  # Check if yellow line is present
                        second_dot_y = i
                        break
                else:
                    second_dot_y = height // 2  # Default to middle if no yellow line is found at that x

                # Draw the second blue dot (following the yellow line at second_dot_x)
                cv2.circle(birdseye_view, (second_dot_x, second_dot_y), 5, (255, 0, 0), -1)

                # Draw a line between the two dots
                cv2.line(birdseye_view, (left_dot_x, left_dot_y), (second_dot_x, second_dot_y), (255, 0, 0), 2)

                # Calculate the distance between the two dots
                distance = np.linalg.norm(np.array([left_dot_x, left_dot_y]) - np.array([second_dot_x, second_dot_y]))

                # Normalize the distance and map to range 0-660 with 330 as the middle
                normalized_distance = (distance / max_distance) * 660
                # Adjust to have a middle point of 330
                adjusted_distance = normalized_distance - 330

                # Draw the adjusted distance on the frame
                cv2.putText(birdseye_view, f"{int(adjusted_distance)} angle", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the outputs
    cv2.imshow("Raw Video", resized_frame)
    cv2.imshow("Bird's-eye View Lines", birdseye_view)
    #cv2.imshow("Bird's-eye View", dilated_yellow_mask)

    # Write the processed frame to the stream
    out_stream.write(birdseye_view)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
out_stream.release()
cv2.destroyAllWindows()