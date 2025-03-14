import cv2
import time
import threading
import queue
from picamera2 import Picamera2
from ultralytics import YOLO
from adafruit_servokit import ServoKit

# Load YOLO model
model = YOLO("yolov8n.pt")  # Use the standard YOLOv8 Nano model

# Initialize ServoKit (PCA9685)
kit = ServoKit(channels=16)
pan_channel, tilt_channel = 0, 1
pan_angle, tilt_angle = 90, 90
kit.servo[pan_channel].angle, kit.servo[tilt_channel].angle = pan_angle, tilt_angle

# Setup PiCamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (960, 960)  # Lower resolution for speed
picam2.preview_configuration.main.format = "RGB888"  # OpenCV expects RGB format
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Queue for frame buffering (with type hint)
frame_queue: queue.Queue[cv2.typing.MatLike] = queue.Queue(maxsize=1)

# Margin before adjusting servos (to avoid unnecessary movements)
MARGIN = 50
ALPHA = 0.2  # Smoothing factor for servo motion
MOVEMENT_THRESHOLD = 1  # Ignore small movements to prevent jitter
TILT_MIN, TILT_MAX = 30, 150

def capture_frames():
    """Continuously capture frames from PiCamera2 and add them to the queue."""
    while True:
        frame = picam2.capture_array()  # Capture frame as NumPy array
        if frame is not None and not frame_queue.full():
            frame_queue.put(frame)

# Start capture thread
threading.Thread(target=capture_frames, daemon=True).start()

while True:
    if not frame_queue.empty():
        frame = frame_queue.get()

        if frame is None:
            print("Warning: Empty frame received, skipping.")
            continue  # Skip processing if frame is empty

        # Object detection
        results = model(frame)

        if not results or len(results[0].boxes.xyxy) == 0:
            print("No object detected, skipping frame.")
            cv2.imshow("Mouse Tracking", frame)  # Show empty frame
            cv2.waitKey(1)  # Allow window to update
            continue  # Skip if no object is detected

        for result in results:
            for box in result.boxes.xyxy:
                x_min, y_min, x_max, y_max = map(int, box)
                obj_center_x, obj_center_y = (x_min + x_max) // 2, (y_min + y_max) // 2

                frame_center_x, frame_center_y = frame.shape[1] // 2, frame.shape[0] // 2

                # Only move if object is significantly off-center
                if abs(obj_center_x - frame_center_x) > MARGIN:
                    pan_angle = min(180, pan_angle + 3) if obj_center_x < frame_center_x else max(0, pan_angle - 3)

                if abs(obj_center_y - frame_center_y) > MARGIN:
                    tilt_angle = min(TILT_MAX, tilt_angle - 2) if obj_center_y < frame_center_y else max(TILT_MIN, tilt_angle + 2)

                # Apply smoothing to servo movement
                smoothed_pan = ALPHA * kit.servo[pan_channel].angle + (1 - ALPHA) * pan_angle
                smoothed_tilt = ALPHA * kit.servo[tilt_channel].angle + (1 - ALPHA) * tilt_angle

                # Only update servos if movement is above threshold (prevent jitter)
                if abs(smoothed_pan - kit.servo[pan_channel].angle) > MOVEMENT_THRESHOLD:
                    kit.servo[pan_channel].angle = smoothed_pan
                if abs(smoothed_tilt - kit.servo[tilt_channel].angle) > MOVEMENT_THRESHOLD:
                    kit.servo[tilt_channel].angle = smoothed_tilt

                print(f"Moving to Pan={smoothed_pan:.1f}, Tilt={smoothed_tilt:.1f}")

                # Draw bounding box
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # Show frame
        cv2.imshow("Mouse Tracking", frame)
        cv2.waitKey(1)  # Allow OpenCV window to refresh

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    
    time.sleep(0.05)
    
cv2.destroyAllWindows()
