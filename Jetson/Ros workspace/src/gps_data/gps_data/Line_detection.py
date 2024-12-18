import cv2
import numpy as np 

cap = cv2.VideoCapture(4)  # Camera index 4 for front camera

if (cap.isOpened()):
    print("Camera opened successfully")
else:
    print("Camera failed to open")

while(cap.isOpened()):
    ret, frame = cap.read()

    if not ret:
        break

    # FILTRID
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
    blur = cv2.GaussianBlur(gray, (5, 5), 0)  # Apply Gaussian Blur

    # ROI: Remove top 1/3 of the screen
    height, width = frame.shape[:2]
    mask = np.zeros_like(gray)
    polygon = np.array([[(0, height), (width, height), (width, height // 3), (0, height // 3)]])
    cv2.fillPoly(mask, [polygon], 255)
    roi = cv2.bitwise_and(blur, mask)

    # Binarize the image
    _, binary = cv2.threshold(roi, 215, 255, cv2.THRESH_BINARY)

    # Apply morphological operations
    kernel = np.ones((5, 5), np.uint8)  #Define a 5x5 kernel for the morphological operations
    binary_cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)  # Close small gaps
    binary_cleaned = cv2.morphologyEx(binary_cleaned, cv2.MORPH_OPEN, kernel)  # Remove small noise

    # Edge detection
    edge = cv2.Canny(binary_cleaned, threshold1=50, threshold2=150)

    # Convert the binary image to color for overlaying
    binary_to_color = cv2.cvtColor(binary_cleaned, cv2.COLOR_GRAY2BGR)
    binary_to_color[np.where((binary_to_color == [255, 255, 255]).all(axis=2))] = [0, 255, 0]
    overlay = cv2.addWeighted(frame, 0.8, binary_to_color, 1, 0)

    # Display results
    if ret:
        cv2.imshow("Raw video", frame)
        cv2.imshow("Binary video", binary_cleaned)
        cv2.imshow("Overlay video", overlay)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()