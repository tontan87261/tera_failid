from ultralytics import YOLO
import torch
import cv2

class RoadLineDetector:
    def __init__(self, model_path='yolov8n.pt', cam_index=0):
        # Ensure the device is set to GPU and raise an error if it's not available
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA GPU is not available. This program requires a GPU.")
        
        self.device = torch.device('cuda')  # Force GPU usage
        print(f"Using device: {self.device}")

        # Load YOLOv8 model onto the GPU
        self.model = YOLO(model_path).to(self.device)

        # Initialize camera
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise IOError(f'Cannot open camera at index {cam_index}')

    def detect_and_annotate(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Resize frame to match YOLOv8 input size
            resized_frame = cv2.resize(frame, (640, 640))

            # Convert to tensor, normalize, and move to GPU
            frame_tensor = torch.from_numpy(resized_frame).permute(2, 0, 1).float() / 255.0
            frame_tensor = frame_tensor.unsqueeze(0).to(self.device)

            # Run YOLOv8 inference on GPU
            results = self.model(frame_tensor)

            # Annotate frame with YOLOv8 detection results
            annotated_frame = results[0].plot()

            # Display annotated frame
            cv2.imshow("YOLOv8 Road Line Detection (GPU)", annotated_frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = RoadLineDetector(model_path='yolov8n.pt', cam_index=0)
    detector.detect_and_annotate()