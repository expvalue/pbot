"""
Overall YOLO — combined general object + fire/smoke detection.

Runs two YOLO models in parallel on each frame:
  - yolov8n (COCO)  → green boxes for regular objects (people, chairs, etc.)
  - fire_yolov8n.pt → red brackets for fire, orange box for smoke

Green  = general object (no fire concern)
Orange = smoke detected
Red    = fire detected (with pulsing border + alert banner)
"""

import os
import time
import math
import urllib.request
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

# ── Models ────────────────────────────────────────────────────────────────────
FIRE_WEIGHTS_URL  = "https://github.com/luminous0219/fire-and-smoke-detection-yolov8/raw/main/weights/best.pt"
FIRE_WEIGHTS_PATH = "fire_yolov8n.pt"

if not os.path.exists(FIRE_WEIGHTS_PATH):
    print(f"Downloading fire/smoke weights to {FIRE_WEIGHTS_PATH}...")
    urllib.request.urlretrieve(FIRE_WEIGHTS_URL, FIRE_WEIGHTS_PATH)
    print("Done.")

general_model = YOLO("yolov8n.pt")          # COCO — 80 everyday classes
fire_model    = YOLO(FIRE_WEIGHTS_PATH)     # fire + smoke

print("General classes:", len(general_model.names), "COCO classes")
print("Fire classes:   ", fire_model.names)

# ── RealSense ─────────────────────────────────────────────────────────────────
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
pipeline.start(config)
align = rs.align(rs.stream.color)

GENERAL_CONF = 0.40     # COCO confidence threshold
FIRE_CONF    = 0.35     # fire/smoke confidence threshold


# ── Visual effects ────────────────────────────────────────────────────────────

def draw_corner_brackets(img, x1, y1, x2, y2, color, thickness=3, length=20):
    """Tactical HUD-style corner brackets instead of a full rectangle."""
    cv2.line(img, (x1, y1), (x1 + length, y1), color, thickness)
    cv2.line(img, (x1, y1), (x1, y1 + length), color, thickness)
    cv2.line(img, (x2, y1), (x2 - length, y1), color, thickness)
    cv2.line(img, (x2, y1), (x2, y1 + length), color, thickness)
    cv2.line(img, (x1, y2), (x1 + length, y2), color, thickness)
    cv2.line(img, (x1, y2), (x1, y2 - length), color, thickness)
    cv2.line(img, (x2, y2), (x2 - length, y2), color, thickness)
    cv2.line(img, (x2, y2), (x2, y2 - length), color, thickness)


def draw_pulsing_border(img, intensity):
    """Red border that pulses based on intensity in [0, 1]."""
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


def draw_label(img, x1, y1, text, color):
    """Filled label background + white text so it stays readable on any scene."""
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 6, y1), color, -1)
    cv2.putText(img, text, (x1 + 3, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)


# ── Main loop ─────────────────────────────────────────────────────────────────
while True:
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        continue

    frame = np.asanyarray(color_frame.get_data())

    # Run both models on the same frame
    gen_results  = general_model(frame, verbose=False)
    fire_results = fire_model(frame,    verbose=False)

    fire_detected  = False
    best_fire_conf = 0.0

    # ── General objects (green) ──────────────────────────────────────────────
    for box in gen_results[0].boxes:
        conf = float(box.conf[0])
        if conf < GENERAL_CONF:
            continue
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = general_model.names[int(box.cls[0])]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        distance = depth_frame.get_distance(cx, cy)

        green = (0, 200, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), green, 2)
        draw_label(frame, x1, y1, f"{label} {conf:.2f} | {distance:.2f}m", green)

    # ── Fire / smoke (red brackets / orange box — drawn on top so it wins) ───
    for box in fire_results[0].boxes:
        conf = float(box.conf[0])
        if conf < FIRE_CONF:
            continue
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = fire_model.names[int(box.cls[0])]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        distance = depth_frame.get_distance(cx, cy)

        is_fire = label.lower() == "fire"
        if is_fire:
            fire_detected  = True
            best_fire_conf = max(best_fire_conf, conf)

        color = (0, 0, 255) if is_fire else (0, 200, 255)
        if is_fire:
            draw_corner_brackets(frame, x1, y1, x2, y2, color, thickness=3, length=22)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        draw_label(frame, x1, y1,
                   f"{label.upper()} {conf:.2f} | {distance:.2f}m", color)

    # ── Alert overlays (only when fire) ──────────────────────────────────────
    if fire_detected:
        pulse     = (math.sin(time.monotonic() * 6.0) + 1) / 2
        intensity = pulse * min(1.0, best_fire_conf * 1.4)
        draw_pulsing_border(frame, intensity)
        draw_alert_banner(frame, f"FIRE DETECTED  |  conf {best_fire_conf:.2f}", intensity)

    cv2.imshow("Overall YOLO", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()