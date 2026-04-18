"""
Fire Fighting Robot — Jetson Nano / Laptop
RealSense camera runs YOLOv8 fire detection.
Steers robot toward fire via serial commands to Arduino,
then resumes the path once fire is extinguished.

States:
  PATROLLING    → no fire, Arduino is following its path
  TRACKING      → fire detected, Jetson steering Arduino toward it
  EXTINGUISHING → close enough, Arduino firing extinguisher
"""

import os
import time
import math
import urllib.request
import pyserial
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

# ── Serial ────────────────────────────────────────────────────────────────────
# Jetson Nano:    '/dev/ttyUSB0'  or  '/dev/ttyACM0'
# Windows laptop: 'COM3'  or  'COM4'  etc.
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE   = 9600

print(f"Connecting to Arduino on {SERIAL_PORT}...")
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
print("Connected.")

def send_command(cmd):
    arduino.write((cmd + '\n').encode())
    print(f"  [→ Arduino] {cmd}")

def read_serial():
    if arduino.in_waiting > 0:
        msg = arduino.readline().decode(errors='ignore').strip()
        if msg:
            print(f"  [← Arduino] {msg}")
        return msg
    return ""

# ── Models ────────────────────────────────────────────────────────────────────
FIRE_WEIGHTS_URL  = "https://github.com/luminous0219/fire-and-smoke-detection-yolov8/raw/main/weights/best.pt"
FIRE_WEIGHTS_PATH = "fire_yolov8n.pt"

if not os.path.exists(FIRE_WEIGHTS_PATH):
    print("Downloading fire/smoke weights...")
    urllib.request.urlretrieve(FIRE_WEIGHTS_URL, FIRE_WEIGHTS_PATH)
    print("Download complete.")

print("Loading models...")
general_model = YOLO("yolov8n.pt")
fire_model    = YOLO(FIRE_WEIGHTS_PATH)
print(f"  General model: {len(general_model.names)} COCO classes")
print(f"  Fire model:    {fire_model.names}")

# ── RealSense camera ──────────────────────────────────────────────────────────
print("Starting RealSense camera...")
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  15)
pipeline.start(config)
align = rs.align(rs.stream.color)
print("Camera ready.")

# ── Tunable parameters ────────────────────────────────────────────────────────
GENERAL_CONF     = 0.40  # COCO detection confidence threshold
FIRE_CONF        = 0.35  # fire/smoke detection confidence threshold
STOP_DISTANCE    = 0.5   # metres — stop and extinguish when fire is this close
FRAME_CENTER_X   = 320   # horizontal midpoint of 640px wide frame
CENTER_TOLERANCE = 60    # px dead-zone — avoids jittery left/right corrections
EXTINGUISH_WAIT  = 6.0   # seconds — how long to wait for Arduino extinguisher cycle

# ── State machine ─────────────────────────────────────────────────────────────
state            = "PATROLLING"
last_cmd_sent    = ""
extinguish_start = 0.0

def set_state(new_state):
    global state
    if state != new_state:
        print(f"[STATE] {state} → {new_state}")
        state = new_state

def send_if_changed(cmd):
    global last_cmd_sent
    if cmd != last_cmd_sent:
        send_command(cmd)
        last_cmd_sent = cmd

# ── Visual helpers ────────────────────────────────────────────────────────────
def draw_corner_brackets(img, x1, y1, x2, y2, color, thickness=3, length=22):
    cv2.line(img, (x1,y1), (x1+length,y1), color, thickness)
    cv2.line(img, (x1,y1), (x1,y1+length), color, thickness)
    cv2.line(img, (x2,y1), (x2-length,y1), color, thickness)
    cv2.line(img, (x2,y1), (x2,y1+length), color, thickness)
    cv2.line(img, (x1,y2), (x1+length,y2), color, thickness)
    cv2.line(img, (x1,y2), (x1,y2-length), color, thickness)
    cv2.line(img, (x2,y2), (x2-length,y2), color, thickness)
    cv2.line(img, (x2,y2), (x2,y2-length), color, thickness)

def draw_pulsing_border(img, intensity):
    h, w = img.shape[:2]
    cv2.rectangle(img, (0,0), (w-1,h-1),
                  (0, 0, int(180 + 75*intensity)),
                  int(6 + 10*intensity))

def draw_alert_banner(img, text, intensity):
    h, w    = img.shape[:2]
    overlay = img.copy()
    cv2.rectangle(overlay, (0,0), (w,48), (0,0,255), -1)
    alpha = 0.55 + 0.35*intensity
    cv2.addWeighted(overlay, alpha, img, 1-alpha, 0, img)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)
    cv2.putText(img, text, ((w-tw)//2, (48+th)//2 - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255,255,255), 2)

def draw_label(img, x1, y1, text, color):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(img, (x1, y1-th-8), (x1+tw+6, y1), color, -1)
    cv2.putText(img, text, (x1+3, y1-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)

def draw_crosshair(img, cx, cy, color):
    cv2.line(img, (cx-12, cy), (cx+12, cy), color, 2)
    cv2.line(img, (cx, cy-12), (cx, cy+12), color, 2)
    cv2.circle(img, (cx, cy), 5, color, -1)

# ── Main loop ─────────────────────────────────────────────────────────────────
print("\nRunning. Press Q in the camera window to quit.\n")

while True:

    read_serial()

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

    # ── General objects (green boxes) ─────────────────────────────────────────
    for box in gen_results[0].boxes:
        conf = float(box.conf[0])
        if conf < GENERAL_CONF:
            continue
        x1,y1,x2,y2 = map(int, box.xyxy[0])
        label = general_model.names[int(box.cls[0])]
        cx,cy = (x1+x2)//2, (y1+y2)//2
        dist  = depth_frame.get_distance(cx, cy)
        green = (0, 200, 0)
        cv2.rectangle(frame, (x1,y1), (x2,y2), green, 2)
        draw_label(frame, x1, y1,
                   f"{label} {conf:.2f} | {dist:.2f}m", green)

    # ── Fire / smoke detection ─────────────────────────────────────────────────
    for box in fire_results[0].boxes:
        conf = float(box.conf[0])
        if conf < FIRE_CONF:
            continue
        x1,y1,x2,y2 = map(int, box.xyxy[0])
        label   = fire_model.names[int(box.cls[0])]
        cx,cy   = (x1+x2)//2, (y1+y2)//2
        dist    = depth_frame.get_distance(cx, cy)
        is_fire = label.lower() == "fire"

        if is_fire and conf > best_fire_conf:
            fire_detected  = True
            best_fire_conf = conf
            fire_cx        = cx
            fire_distance  = dist

        color = (0, 0, 255) if is_fire else (0, 200, 255)
        if is_fire:
            draw_corner_brackets(frame, x1,y1,x2,y2, color)
            draw_crosshair(frame, cx, cy, color)
        else:
            cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)

        dist_str = f"{dist:.2f}m" if dist > 0 else "---"
        draw_label(frame, x1, y1,
                   f"{label.upper()} {conf:.2f} | {dist_str}", color)

    # ── State machine ──────────────────────────────────────────────────────────
    if state == "PATROLLING":
        if fire_detected:
            set_state("TRACKING")

    elif state == "TRACKING":
        if not fire_detected:
            send_if_changed("RESUME")
            set_state("PATROLLING")

        elif fire_distance is not None and 0 < fire_distance <= STOP_DISTANCE:
            send_command("FIRE_STOP")
            last_cmd_sent    = "FIRE_STOP"
            extinguish_start = time.time()
            set_state("EXTINGUISHING")

        else:
            offset = fire_cx - FRAME_CENTER_X
            if offset > CENTER_TOLERANCE:
                send_if_changed("FIRE_RIGHT")
            elif offset < -CENTER_TOLERANCE:
                send_if_changed("FIRE_LEFT")
            else:
                send_if_changed("FIRE_AHEAD")

    elif state == "EXTINGUISHING":
        if time.time() - extinguish_start >= EXTINGUISH_WAIT:
            send_command("RESUME")
            last_cmd_sent = "RESUME"
            set_state("PATROLLING")

    # ── Alert overlays ─────────────────────────────────────────────────────────
    if fire_detected:
        pulse     = (math.sin(time.monotonic() * 6.0) + 1) / 2
        intensity = pulse * min(1.0, best_fire_conf * 1.4)
        draw_pulsing_border(frame, intensity)
        dist_str = f"{fire_distance:.2f}m" if fire_distance else "---"
        draw_alert_banner(frame,
            f"FIRE  conf:{best_fire_conf:.2f}  dist:{dist_str}", intensity)

    # ── HUD overlays ───────────────────────────────────────────────────────────
    state_color = {
        "PATROLLING":    (0, 200, 0),
        "TRACKING":      (0, 165, 255),
        "EXTINGUISHING": (0, 0, 255),
    }.get(state, (255,255,255))

    cv2.putText(frame, f"STATE: {state}", (10, 460),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)

    # Centre aim line
    cv2.line(frame, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, 480),
             (80, 80, 80), 1)

    cv2.imshow("Fire Robot", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        send_command("RESUME")
        break

# ── Cleanup ────────────────────────────────────────────────────────────────────
pipeline.stop()
arduino.close()
cv2.destroyAllWindows()
print("Shut down cleanly.")