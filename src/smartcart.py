import lgpio
import time
import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
from adafruit_servokit import ServoKit
from lcdDriver import LCD
import numpy as np
from tensorflow.keras.models import load_model
import tensorflow.lite as tflite
import socket

# GPIO Pins
ECHO_PIN = 9
TRIG_PIN = 10
LCD_RS = 25
LCD_EN = 24
LCD_D4 = 23
LCD_D5 = 17
LCD_D6 = 18
LCD_D7 = 22
LCD_BACKLIGHT = 4

class SmartCart:
    # FSM States
    STANDBY, PRIMARY, TRACKING, SECONDARY, CLASSIFY, TRANSMIT = range(6)

    def __init__(self):
        self.state = self.STANDBY
        self.h = lgpio.gpiochip_open(0)
        self.ultrasonic_init()
        self.lcd = LCD(self.h, LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_BACKLIGHT)
        self.lcd.lcd_message("...SmartCART...")
        self.ultrasonic_init()
        self.camera_init()
        self.mount_init()
        self.yolo_init()
        self.cnn_init()
        self.tcp_init()
        self.trackStart = None

    def ultrasonic_init(self):
        lgpio.gpio_claim_output(self.h, TRIG_PIN)
        lgpio.gpio_claim_input(self.h, ECHO_PIN)
        
    def camera_init(self):
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (960, 960)
        self.picam2.preview_configuration.main.format = "RGB888"
        self.picam2.preview_configuration.align()
        self.picam2.configure("preview")
        self.picam2.start()
    
    def mount_init(self):
        self.kit = ServoKit(channels=16)
        self.pan_channel, self.tilt_channel = 0, 1
        self.pan_angle, self.tilt_angle = 90, 90
        self.kit.servo[self.pan_channel].angle = self.pan_angle
        self.kit.servo[self.tilt_channel].angle = self.tilt_angle
        
    def yolo_init(self):
        self.model = YOLO("yolov8n.pt")
        self.ignoreClasses = ["person"]
        self.ignoreIDs = [classID for classID, name in self.model.names.items() if name in  self.ignoreClasses]

    def cnn_init(self):
        self.interpreter = tflite.Interpreter(model_path="smartcart_classifier.tflite")
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.class_names = ['book', 'grey-mouse', 'keyboard', 'padlock', 'pencil-case', 'pink-mouse']
    
    def tcp_init(self):
        self.serverIP = '10.242.254.39'
        self.port = 5000

    def detectObject(self, threshold = 10):
        lgpio.gpio_write(self.h, TRIG_PIN, 0)
        time.sleep(0.1)
        lgpio.gpio_write(self.h, TRIG_PIN, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.h, TRIG_PIN, 0)

        start = time.time()
        timeout = start + 0.02  

        while lgpio.gpio_read(self.h, ECHO_PIN) == 0:
            if time.time() > timeout:
                return False
            
        start = time.time()
        while lgpio.gpio_read(self.h, ECHO_PIN) == 1:
            stop = time.time()
            if stop > timeout:
                return False
            
        distance = ((stop - start) * 33000) / 2
        print(f"Measured Distance: {distance:.2f} cm")
        return distance < threshold
    
    def capturePrimary(self, n = 3):
        frameBuffer = []
        try:
            while True:
                self.frame = self.picam2.capture_array()
                cv2.imshow("Live Preview", self.frame)

                key = cv2.waitKey(1) & 0xFF

                if key == ord("u"):
                    print("'U' pressed! Capturing initial image...")

                    for _ in range(n):
                        frame = self.picam2.capture_array()
                        results = self.model(frame)

                        if results[0].boxes.conf.numel() > 0:
                            confidences = results[0].boxes.conf.tolist()
                            maxConfidence = max(confidences)
                            frameBuffer.append((frame, results[0], maxConfidence))

                            if len(frameBuffer) > n:
                                frameBuffer.pop(0)

                    if frameBuffer:
                        bestFrame, bestResult, _ = max(frameBuffer, key=lambda x: x[2])

                        bestBoxes = bestResult.boxes.xyxy
                        bestClassIDs = bestResult.boxes.cls

                        processedFrame = bestFrame.copy()

                        detected = False

                        for box, classID in zip(bestBoxes, bestClassIDs):
                            if int(classID) not in self.ignoreIDs:
                                x_min, y_min, x_max, y_max = map(int, box)

                                cv2.rectangle(processedFrame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                                label = f"{self.model.names[int(classID)]}"
                                cv2.putText(processedFrame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                                cropped_object = bestFrame[y_min:y_max, x_min:x_max]
                                cv2.imwrite("primary.jpg", cropped_object)
                                print("Saving primary image... \n")

                                detected = True

                        cv2.imshow("YOLO Detection", processedFrame)
                        frameBuffer.clear()

                        if detected:
                            cv2.destroyAllWindows()
                            return True

                    print("No valid objects detected.")
                    return False

                elif key == ord("q"):
                    print("User exited capture mode. \n")
                    return False
                
                time.sleep(0.1)

        finally:
            cv2.destroyAllWindows()
            
    def trackObject(self):
        MARGIN = 50
        ALPHA = 0.2
        MOVEMENT_THRESHOLD = 1
        TILT_MIN, TILT_MAX = 30, 150
        STABILITY_DURATION = 3

        while True:
            frame = self.picam2.capture_array()
            results = self.model(frame)

            cv2.imshow("Live Tracking", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            steady = True

            for result in results:
                for box in result.boxes.xyxy:
                    x_min, y_min, x_max, y_max = map(int, box)
                    obj_center_x, obj_center_y = (x_min + x_max) // 2, (y_min + y_max) // 2
                    frame_center_x, frame_center_y = frame.shape[1] // 2, frame.shape[0] // 2

                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.drawMarker(frame, (obj_center_x, obj_center_y), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)

                    if abs(obj_center_x - frame_center_x) > MARGIN:
                        self.pan_angle = min(180, self.pan_angle + 3) if obj_center_x < frame_center_x else max(0, self.pan_angle - 3)
                        steady = False

                    if abs(obj_center_y - frame_center_y) > MARGIN:
                        self.tilt_angle = min(TILT_MAX, self.tilt_angle - 2) if obj_center_y < frame_center_y else max(TILT_MIN, self.tilt_angle + 2)
                        steady = False

                    smoothed_pan = ALPHA * self.kit.servo[self.pan_channel].angle + (1 - ALPHA) * self.pan_angle
                    smoothed_tilt = ALPHA * self.kit.servo[self.tilt_channel].angle + (1 - ALPHA) * self.tilt_angle

                    if abs(smoothed_pan - self.kit.servo[self.pan_channel].angle) > MOVEMENT_THRESHOLD:
                        self.kit.servo[self.pan_channel].angle = smoothed_pan
                        steady = False

                    if abs(smoothed_tilt - self.kit.servo[self.tilt_channel].angle) > MOVEMENT_THRESHOLD:
                        self.kit.servo[self.tilt_channel].angle = smoothed_tilt
                        steady = False

                    print(f"Tracking: Pan={smoothed_pan:.1f}, Tilt={smoothed_tilt:.1f}")

                    if steady:
                        if self.trackStart is None:
                            self.trackStart = time.time()
                        elif time.time() - self.trackStart >= STABILITY_DURATION:
                            print("Object tracking stable.")
                            cv2.destroyAllWindows()
                            return True
                    else:
                        self.trackStart = None

                    self.last_pan, self.last_tilt = smoothed_pan, smoothed_tilt

            cv2.imshow("Live Tracking", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()
        return False 
    
    def captureSecondary(self, filename = "secondary.jpg"):
        frame = self.picam2.capture_array()
        results = self.model(frame)
        
        if not results or len(results[0].boxes.xyxy) == 0:
            print("No object detected in secondary image.\n")
            return False

        for result in results:
            for box in result.boxes.xyxy:
                x_min, y_min, x_max, y_max = map(int, box)

                cropped_object = frame[y_min:y_max, x_min:x_max]
                
                if cropped_object.size != 0:
                    cv2.imwrite(filename, cropped_object)
                    print(f"Cropped object saved as {filename}\n")
                    return True

        print("No valid object found for cropping in secondary image.\n")
        return False
    
    def classifyObject(self, image_path):
        image = cv2.imread(image_path)
        image = cv2.resize(image, (128, 128))
        image = np.expand_dims(image, axis=0).astype(np.float32) / 255.0

        self.interpreter.set_tensor(self.input_details[0]['index'], image)
        self.interpreter.invoke()
        predictions = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        top_index = np.argmax(predictions)
        top_class = self.class_names[top_index]
        confidence = predictions[top_index]

        print(f"Classified as: {top_class} (Confidence: {confidence:.2f})")
        return top_class, confidence
    
    def transmitClassification(self):
        message = f"Detected Object: {self.final_class}"
        
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((self.serverIP, self.port))
            client_socket.send(message.encode('utf-8'))
            
            response = client_socket.recv(1024).decode('utf-8')
            print(f"Server response: {response}")
        
        except Exception as e:
            print(f"Error: {e}")
        
        finally:
            client_socket.close()
        
    def handleState(self):
        match self.state:
            case self.STANDBY:
                print("STATE: STANDBY")
                self.lcd.lcd_message("STATE: STANDBY")
                if self.detectObject():
                    self.state = self.PRIMARY
                else:
                    self.state = self.STANDBY

            case self.PRIMARY:
                print("STATE: PRIMARY")
                self.lcd.lcd_message("STATE: PRIMARY")
                if self.capturePrimary():
                    self.state = self.TRACKING
                else:
                    self.state = self.STANDBY

            case self.TRACKING:
                print("STATE: TRACKING")
                self.lcd.lcd_message("STATE: TRACKING")
                if self.trackObject():
                    self.state = self.SECONDARY
                else:
                    self.state = self.STANDBY  

            case self.SECONDARY:
                print("STATE: SECONDARY")
                self.lcd.lcd_message("STATE: SECONDARY")
                if self.captureSecondary():
                    self.state = self.CLASSIFY
                else:
                    self.state = self.STANDBY  

            case self.CLASSIFY:
                print("STATE: CLASSIFY")
                self.lcd.lcd_message("STATE: CLASSIFY")

                primary_class, primary_conf = self.classifyObject("primary.jpg")
                secondary_class, secondary_conf = self.classifyObject("secondary.jpg")
                
                
                if primary_conf > secondary_conf:
                    self.final_class = primary_class
                else:
                    self.final_class = secondary_class

                print(f"Final Classification: {self.final_class}")
                self.lcd.lcd_message("Final Class:", self.final_class)

                self.state = self.TRANSMIT

            case self.TRANSMIT:
                print("STATE: TRANSMIT")
                self.lcd.lcd_message("STATE: TRANSMIT")
                self.transmitClassification()
                self.state = self.STANDBY  

            case _:
                print("Invalid state!")

smartcart = SmartCart()
while True:
    smartcart.handleState()
    time.sleep(3)

    
    
    