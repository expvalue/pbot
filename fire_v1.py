import os
import time
import math
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
print("Classes:", model.names)

# ── RealSense ─────────────────────────────────────────────────────────────────
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
pipeline.start(config)
align = rs.align(rs.stream.color)

CONF_THRESHOLD = 0.35


# ── Visual effects ────────────────────────────────────────────────────────────

def draw_corner_brackets(img, x1, y1, x2, y2, color, thickness=3, length=20):
    """Tactical HUD-style corner brackets instead of a full rectangle."""
    # top-left
    cv2.line(img, (x1, y1), (x1 + length, y1), color, thickness)
    cv2.line(img, (x1, y1), (x1, y1 + length), color, thickness)
    # top-right
    cv2.line(img, (x2, y1), (x2 - length, y1), color, thickness)
    cv2.line(img, (x2, y1), (x2, y1 + length), color, thickness)
    # bottom-left
    cv2.line(img, (x1, y2), (x1 + length, y2), color, thickness)
    cv2.line(img, (x1, y2), (x1, y2 - length), color, thickness)
    # bottom-right
    cv2.line(img, (x2, y2), (x2 - length, y2), color, thickness)
    cv2.line(img, (x2, y2), (x2, y2 - length), color, thickness)


def draw_pulsing_border(img, intensity):
    """Red border around the frame that pulses based on intensity in [0, 1]."""
    h, w = img.shape[:2]
    thickness = int(6 + 10 * intensity)
    red       = (0, 0, int(180 + 75 * intensity))
    cv2.rectangle(img, (0, 0), (w - 1, h - 1), red, thickness)


def draw_alert_banner(img, text, intensity):
    """Semi-transparent red banner across the top with alert text."""
    h, w = img.shape[:2]
    banner_h = 48
    overlay  = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, banner_h), (0, 0, 255), -1)
    alpha = 0.55 + 0.35 * intensity
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    tx = (w - tw) // 2
    ty = (banner_h + th) // 2 - 2
    cv2.putText(img, text, (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)


# ── Main loop ─────────────────────────────────────────────────────────────────

while True:
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        continue

    frame = np.asanyarray(color_frame.get_data())
    results = model(frame, verbose=False)

    fire_detected    = False
    best_fire_conf   = 0.0

    for box in results[0].boxes:
        conf = float(box.conf[0])
        if conf < CONF_THRESHOLD:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = model.names[int(box.cls[0])]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        distance = depth_frame.get_distance(cx, cy)

        is_fire = label.lower() == "fire"
        if is_fire:
            fire_detected  = True
            best_fire_conf = max(best_fire_conf, conf)

        # Corner brackets for fire, classic rectangle for smoke
        color = (0, 0, 255) if is_fire else (0, 200, 255)
        if is_fire:
            draw_corner_brackets(frame, x1, y1, x2, y2, color, thickness=3, length=22)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Label background (so white text stays readable on any scene)
        tag = f"{label.upper()} {conf:.2f} | {distance:.2f}m"
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 6, y1), color, -1)
        cv2.putText(frame, tag, (x1 + 3, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # ── Alert overlays (only when fire is detected) ──────────────────────────
    if fire_detected:
        # Smooth 0→1→0 pulse driven by time, scaled by detection confidence
        pulse     = (math.sin(time.monotonic() * 6.0) + 1) / 2   # 0..1
        intensity = pulse * min(1.0, best_fire_conf * 1.4)

        draw_pulsing_border(frame, intensity)
        draw_alert_banner(frame, f"FIRE DETECTED  |  conf {best_fire_conf:.2f}", intensity)

    cv2.imshow("Fire Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
