import os
import shutil
import random

random.seed(42)

img_dir = "dataset/images/raw"
all_imgs = [f for f in os.listdir(img_dir) if f.endswith('.jpg') or f.endswith('.png')]
random.shuffle(all_imgs)

split_idx = int(len(all_imgs) * 0.8)
train_imgs = all_imgs[:split_idx]
val_imgs = all_imgs[split_idx:]

for phase, imgs in [("train", train_imgs), ("val", val_imgs)]:
    os.makedirs(f"dataset/images/{phase}", exist_ok=True)
    os.makedirs(f"dataset/labels/{phase}", exist_ok=True)
    for f in imgs:
        shutil.copy(f"dataset/images/raw/{f}", f"dataset/images/{phase}/{f}")

print(f"训练集: {len(train_imgs)} 张, 验证集: {len(val_imgs)} 张")