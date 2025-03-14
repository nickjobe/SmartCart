import cv2
import time
from picamera2 import Picamera2
from ultralytics import YOLO

# Set up the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280, 1280)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load YOLO model for object detection
model = YOLO("yolov10n.pt")

classes = model.names
ignoreClasses = ["person"]
ignoreIDs = []

for classID, name in classes.items():
    if name in ignoreClasses:
        ignoreIDs.append(classID)

# Frame buffer to store captured frames for best selection
n = 5
frameBuffer = []

print("\n Menu: U - Capture frame, Q - Quit \n")

try:
    while True:
        # Creating live preview so we know when to take image
        frame = picam2.capture_array()
        cv2.imshow("Live Preview", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("u"):
            print("'U' pressed! Capturing frames for detection...")

            for _ in range(n):
                frame = picam2.capture_array()
                results = model(frame)

                if results[0].boxes.conf.numel() > 0: # Check if something detected
                    confidences = results[0].boxes.conf.tolist()
                    maxConfidence = max(confidences) # Get the highest confidence score

                    frameBuffer.append((frame, results[0], maxConfidence))

                    if len(frameBuffer) > n:
                        frameBuffer.pop(0)
                        
                    if len(frameBuffer) > 0:
                        bestFrame, bestResult, _ = max(frameBuffer, key=lambda x: x[2])  # Pick highest confidence

                        bestBoxes = bestResult.boxes.xyxy  # Extract bounding boxes
                        bestClassIDs = bestResult.boxes.cls  # Extract class IDs

                        processedFrame = bestFrame.copy()

                        for box, classID in zip(bestBoxes, bestClassIDs):  # Iterate correctly
                            if int(classID) not in ignoreIDs:
                                x_min, y_min, x_max, y_max = map(int, box)

                                cv2.rectangle(processedFrame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                                label = f"{model.names[int(classID)]}"
                                cv2.putText(processedFrame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                                # Crop and save detected object
                                cropped_object = bestFrame[y_min:y_max, x_min:x_max]
                                cv2.imwrite("template-tracking.jpg", cropped_object)
                                print("Template saved! \n")

                    cv2.imshow("YOLO Detection", processedFrame)
                    frameBuffer.clear()
                    
        if key == ord("q"):
            break
        
        time.sleep(0.1)

finally:
    cv2.destroyAllWindows()
