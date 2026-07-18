from ultralytics import YOLO
model = YOLO('runs/train/target_detector/weights/best.pt')
model.export(format='onnx', imgsz=640)
print("ONNX exported to models/target.onnx")