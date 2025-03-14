from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO("yolov8n.pt")

# Export the model to NCNN format
model.export(format="ncnn", imgsz=640)  # creates 'yolov8n_ncnn_model'