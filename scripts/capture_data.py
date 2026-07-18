import cv2
import os
import time

os.makedirs("dataset/images/raw", exist_ok=True)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("=== 数据采集工具 ===")
print("按 [C] 键拍照保存")
print("按 [Q] 键退出")
print("提示：请将目标放在 4-6m 距离，变换不同角度和光照")

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败，请检查 /dev/video0 是否存在")
        break

    cv2.putText(frame, f"Count: {count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("Capture", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        filename = f"dataset/images/raw/{int(time.time())}_{count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"[OK] 已保存: {filename}")
        count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"采集完成！共采集 {count} 张图片。")