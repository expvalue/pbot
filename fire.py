import os
import urllib.request
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

# ── Download fire/smoke weights once ──────────────────────────────────────────
WEIGHTS_URL  = "https://github.com/luminous0219/fire-and-smoke-detection-yolov8/raw/main/weights/best.pt"
WEIGHTS_PATH = "fire_yolov8n.pt"

if not os.path.exists(WEIGHTS_PATH):
    print(f"Downloading fire/smoke weights to {WEIGHTS_PATH}...")
    urllib.request.urlretrieve(WEIGHTS_URL, WEIGHTS_PATH)
    print("Done.")

model = YOLO(WEIGHTS_PATH)
print("Classes:", model.names)   # should be {0: 'fire', 1: 'smoke'} or similar

# ── RealSense ─────────────────────────────────────────────────────────────────
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
pipeline.start(config)
align = rs.align(rs.stream.color)

CONF_THRESHOLD = 0.35

while True:
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        continue

    frame = np.asanyarray(color_frame.get_data())
    results = model(frame, verbose=False)

    for box in results[0].boxes:
        conf = float(box.conf[0])
        if conf < CONF_THRESHOLD:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = model.names[int(box.cls[0])]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        distance = depth_frame.get_distance(cx, cy)

        # red for fire, yellow-ish for smoke
        color = (0, 0, 255) if label.lower() == "fire" else (0, 200, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{label} {conf:.2f} | {distance:.2f}m",
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow("Fire Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()