from ultralytics import YOLO
import os

def main():
    # 加载预训练模型
    model = YOLO('yolov8n.pt')
    
    # 训练（请确保 dataset/data.yaml 存在）
    model.train(
        data='dataset/data.yaml',
        epochs=100,
        imgsz=640,
        batch=16,
        device=0,
        project='runs/train',
        name='target_detector',
        patience=20
    )
    print("Training finished!")

if __name__ == "__main__":
    main()