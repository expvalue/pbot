"""
Fire Fighting Robot — Main Python Script
Runs on laptop (or Jetson later).
Camera detects fire → state machine steers robot toward it → extinguishes.
"""

import os
import time
import math
import urllib.request
import serial
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

# ── Serial — change 'COM3' to your Arduino port ───────────────────────────────
# Windows: 'COM3', 'COM4', etc.
# Mac/Linux: '/dev/ttyUSB0' or '/dev/ttyACM0'
SERIAL_PORT = 'COM3'

arduino = serial.Serial(SERIAL_PORT, 9600, timeout=1)
time.sleep(2)   # wait for Arduino to reset after USB connect
print(f"Connected to Arduino on {SERIAL_PORT}")

def send_command(cmd):
    arduino.write((cmd + '\n').encode())
    print(f"[CMD] {cmd}")

# ── Models ────────────────────────────────────────────────────────────────────
FIRE_WEIGHTS_URL  = "https://github.com/luminous0219/fire-and-smoke-detection-yolov8/raw/main/weights/best.pt"
FIRE_WEIGHTS_PATH = "fire_yolov8n.pt"

if not os.path.exists(FIRE_WEIGHTS_PATH):
    print(f"Downloading fire/smoke weights...")
    urllib.request.urlretrieve(FIRE_WEIGHTS_URL, FIRE_WEIGHTS_PATH)
    print("Done.")

general_model = YOLO("yolov8n.pt")
fire_model    = YOLO(FIRE_WEIGHTS_PATH)
print(f"General model: {len(general_model.names)} COCO classes")
print(f"Fire model classes: {fire_model.names}")

# ── RealSense ─────────────────────────────────────────────────────────────────
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  15)
pipeline.start(config)
align = rs.align(rs.stream.color)
print("RealSense camera started.")

# ── Tunable parameters ────────────────────────────────────────────────────────
GENERAL_CONF      = 0.40   # confidence threshold for COCO objects
FIRE_CONF         = 0.35   # confidence threshold for fire/smoke
STOP_DISTANCE     = 0.6    # metres — stop & extinguish when fire is this close
FRAME_CENTER_X    = 320    # horizontal centre of 640px frame
CENTER_TOLERANCE  = 60     # px dead-zone to avoid left/right jitter

# ── State machine ─────────────────────────────────────────────────────────────
# PATROLLING    → no fire, robot idle
# TRACKING      → fire seen, robot steering toward it
# EXTINGUISHING → close enough, extinguisher firing
state = "PATROLLING"

# ── Visual helpers ────────────────────────────────────────────────────────────
def draw_corner_brackets(img, x1, y1, x2, y2, color, thickness=3, length=22):
    cv2.line(img, (x1, y1), (x1+length, y1), color, thickness)
    cv2.line(img, (x1, y1), (x1, y1+length), color, thickness)
    cv2.line(img, (x2, y1), (x2-length, y1), color, thickness)
    cv2.line(img, (x2, y1), (x2, y1+length), color, thickness)
    cv2.line(img, (x1, y2), (x1+length, y2), color, thickness)
    cv2.line(img, (x1, y2), (x1, y2-length), color, thickness)
    cv2.line(img, (x2, y2), (x2-length, y2), color, thickness)
    cv2.line(img, (x2, y2), (x2, y2-length), color, thickness)

def draw_pulsing_border(img, intensity):
    h, w = img.shape[:2]
    thickness = int(6 + 10 * intensity)
    color     = (0, 0, int(180 + 75 * intensity))
    cv2.rectangle(img, (0, 0), (w-1, h-1), color, thickness)

def draw_alert_banner(img, text, intensity):
    h, w    = img.shape[:2]
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 48), (0, 0, 255), -1)
    alpha = 0.55 + 0.35 * intensity
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    cv2.putText(img, text, ((w-tw)//2, (48+th)//2 - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

def draw_label(img, x1, y1, text, color):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(img, (x1, y1-th-8), (x1+tw+6, y1), color, -1)
    cv2.putText(img, text, (x1+3, y1-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

# ── Main loop ─────────────────────────────────────────────────────────────────
print("Running. Press Q in the camera window to quit.")

while True:
    frames  = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        continue

    frame = np.asanyarray(color_frame.get_data())

    gen_results  = general_model(frame, verbose=False)
    fire_results = fire_model(frame,    verbose=False)

    fire_detected  = False
    best_fire_conf = 0.0
    fire_cx        = None
    fire_distance  = None

    # ── General objects (green boxes) ────────────────────────────────────────
    for box in gen_results[0].boxes:
        conf = float(box.conf[0])
        if conf < GENERAL_CONF:
            continue
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = general_model.names[int(box.cls[0])]
        cx, cy = (x1+x2)//2, (y1+y2)//2
        dist   = depth_frame.get_distance(cx, cy)
        green  = (0, 200, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), green, 2)
        draw_label(frame, x1, y1, f"{label} {conf:.2f} | {dist:.2f}m", green)

    # ── Fire / smoke detection ────────────────────────────────────────────────
    for box in fire_results[0].boxes:
        conf = float(box.conf[0])
        if conf < FIRE_CONF:
            continue
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label   = fire_model.names[int(box.cls[0])]
        cx, cy  = (x1+x2)//2, (y1+y2)//2
        dist    = depth_frame.get_distance(cx, cy)
        is_fire = label.lower() == "fire"

        # Track the highest-confidence fire detection this frame
        if is_fire and conf > best_fire_conf:
            fire_detected  = True
            best_fire_conf = conf
            fire_cx        = cx
            fire_distance  = dist

        color = (0, 0, 255) if is_fire else (0, 200, 255)
        if is_fire:
            draw_corner_brackets(frame, x1, y1, x2, y2, color)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        draw_label(frame, x1, y1,
                   f"{label.upper()} {conf:.2f} | {dist:.2f}m", color)

    # ── State machine — decides what to send to Arduino ───────────────────────
    if state == "EXTINGUISHING":
        # Arduino is running its 5-second extinguisher cycle autonomously.
        # Python just waits, then returns to patrol.
        time.sleep(5.5)
        send_command("STOP")
        state = "PATROLLING"
        print("[STATE] PATROLLING")

    elif state == "TRACKING":
        if not fire_detected:
            send_command("STOP")
            state = "PATROLLING"
            print("[STATE] PATROLLING — fire lost")

        elif fire_distance is not None and 0 < fire_distance <= STOP_DISTANCE:
            send_command("EXTINGUISH")
            state = "EXTINGUISHING"
            print("[STATE] EXTINGUISHING")

        else:
            # Steer toward the fire using its horizontal position in frame
            offset = fire_cx - FRAME_CENTER_X
            if offset > CENTER_TOLERANCE:
                send_command("RIGHT")
            elif offset < -CENTER_TOLERANCE:
                send_command("LEFT")
            else:
                send_command("FORWARD")

    elif state == "PATROLLING":
        if fire_detected:
            state = "TRACKING"
            print("[STATE] TRACKING — fire detected!")

    # ── Alert overlays ────────────────────────────────────────────────────────
    if fire_detected:
        pulse     = (math.sin(time.monotonic() * 6.0) + 1) / 2
        intensity = pulse * min(1.0, best_fire_conf * 1.4)
        draw_pulsing_border(frame, intensity)
        dist_str = f"{fire_distance:.2f}m" if fire_distance else "---"
        draw_alert_banner(frame,
            f"FIRE DETECTED  |  conf {best_fire_conf:.2f}  |  {dist_str}", intensity)

    # ── State label on screen ─────────────────────────────────────────────────
    state_color = {
        "PATROLLING":    (0, 200, 0),
        "TRACKING":      (0, 165, 255),
        "EXTINGUISHING": (0, 0, 255),
    }.get(state, (255, 255, 255))
    cv2.putText(frame, f"STATE: {state}", (10, 470),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)

    cv2.imshow("Fire Robot", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        send_command("STOP")
        break

# ── Cleanup ───────────────────────────────────────────────────────────────────
pipeline.stop()
arduino.close()
cv2.destroyAllWindows()
print("Shut down cleanly.")
