"""
firebot_v2.py — Fire-detection bot with staggered extinguisher trigger

Drives forward continuously.
Fire detected → stops immediately.
Fire seen for 1.3s → servo 1 fires.
Fire seen for 2.0s → servo 2 fires.
No distance check — any fire on screen counts.
0.5s grace period — detection can flicker without resetting the timer.

Arduino sketch: pbot_v2.ino

Usage:
    python firebot_v2.py
"""

import os, sys, time, math, urllib.request

print("[firebot] Importing libraries...", flush=True)

try:
    import serial
except ImportError:
    serial = None
    print("[firebot] WARNING: pyserial missing — vision only, no motors", flush=True)

import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

print("[firebot] Imports OK.", flush=True)

# ══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════════════

ARDUINO_PORT = os.environ.get("ARDUINO_PORT", "COM10").strip()
ARDUINO_BAUD = 9600

FIRE_WEIGHTS_URL  = "https://github.com/luminous0219/fire-and-smoke-detection-yolov8/raw/main/weights/best.pt"
FIRE_WEIGHTS_PATH = "fire_yolov8n.pt"

FIRE_CONF    = 0.35
GENERAL_CONF = 0.40

SERVO1_HOLD_SEC = 1.3     # servo 1 fires after 1.3s of fire
SERVO2_HOLD_SEC = 2.0     # servo 2 fires after 2.0s of fire
GRACE_SEC       = 0.5     # detection can drop out this long without resetting


# ══════════════════════════════════════════════════════════════════════════════
#  ARDUINO SERIAL
# ══════════════════════════════════════════════════════════════════════════════

def open_arduino():
    if not ARDUINO_PORT:
        print("[firebot] ARDUINO_PORT not set — vision only.", flush=True)
        return None
    if serial is None:
        return None
    try:
        ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.05)
        time.sleep(2.0)
        ser.reset_input_buffer()
        print(f"[firebot] Arduino connected on {ARDUINO_PORT}", flush=True)
        return ser
    except OSError as e:
        print(f"[firebot] Could not open {ARDUINO_PORT}: {e}", flush=True)
        return None

arduino = open_arduino()

def send_cmd(cmd: str):
    if arduino is None or cmd not in ("F", "S", "1", "2"):
        return
    arduino.write(cmd.encode())


# ══════════════════════════════════════════════════════════════════════════════
#  YOLO MODELS
# ══════════════════════════════════════════════════════════════════════════════

if not os.path.exists(FIRE_WEIGHTS_PATH):
    print(f"[firebot] Downloading fire weights → {FIRE_WEIGHTS_PATH}...", flush=True)
    urllib.request.urlretrieve(FIRE_WEIGHTS_URL, FIRE_WEIGHTS_PATH)

general_model = YOLO("yolov8n.pt")
fire_model    = YOLO(FIRE_WEIGHTS_PATH)
print(f"[firebot] Fire model classes: {fire_model.names}", flush=True)


# ══════════════════════════════════════════════════════════════════════════════
#  REALSENSE
# ══════════════════════════════════════════════════════════════════════════════

pipeline = rs.pipeline()
rs_cfg   = rs.config()
rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  15)
pipeline.start(rs_cfg)
align = rs.align(rs.stream.color)
print("[firebot] RealSense streaming.", flush=True)


# ══════════════════════════════════════════════════════════════════════════════
#  HUD DRAWING
# ══════════════════════════════════════════════════════════════════════════════

def draw_corner_brackets(img, x1, y1, x2, y2, color, thickness=3, length=22):
    for (sx,sy),(dx,dy) in [
        ((x1,y1),(1,0)),((x1,y1),(0,1)),((x2,y1),(-1,0)),((x2,y1),(0,1)),
        ((x1,y2),(1,0)),((x1,y2),(0,-1)),((x2,y2),(-1,0)),((x2,y2),(0,-1)),
    ]:
        cv2.line(img,(sx,sy),(sx+dx*length,sy+dy*length),color,thickness)

def draw_pulsing_border(img, intensity):
    h, w = img.shape[:2]
    cv2.rectangle(img,(0,0),(w-1,h-1),(0,0,int(180+75*intensity)),int(6+10*intensity))

def draw_alert_banner(img, text, intensity):
    h, w = img.shape[:2]
    overlay = img.copy()
    cv2.rectangle(overlay,(0,0),(w,48),(0,0,255),-1)
    cv2.addWeighted(overlay, 0.55+0.35*intensity, img, 0.45-0.35*intensity+0.35, 0, img)
    (tw,th),_ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    cv2.putText(img, text, ((w-tw)//2,(48+th)//2-2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)

def draw_label(img, x1, y1, text, color):
    (tw,th),_ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(img,(x1,y1-th-8),(x1+tw+6,y1),color,-1)
    cv2.putText(img, text, (x1+3,y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════════════════════════════

print("\n[firebot] RUNNING", flush=True)
print(f"[firebot]   Servo 1 at {SERVO1_HOLD_SEC}s  |  Servo 2 at {SERVO2_HOLD_SEC}s", flush=True)
print("[firebot]   Press 'q' in the video window to quit.\n", flush=True)

fire_start     = None     # when fire was first seen
last_fire_time = None     # last frame fire was seen (for grace period)
servo1_fired   = False
servo2_fired   = False

try:
    while True:
        frames  = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        fh, fw = frame.shape[:2]

        gen_results  = general_model(frame, verbose=False)
        fire_results = fire_model(frame,    verbose=False)

        fire_detected  = False
        best_fire_conf = 0.0

        # ── General objects (green) ───────────────────────────────────────
        for box in gen_results[0].boxes:
            conf = float(box.conf[0])
            if conf < GENERAL_CONF:
                continue
            x1,y1,x2,y2 = map(int, box.xyxy[0])
            label = general_model.names[int(box.cls[0])]
            cx, cy = (x1+x2)//2, (y1+y2)//2
            dist = depth_frame.get_distance(cx, cy)
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,200,0),2)
            draw_label(frame, x1, y1, f"{label} {conf:.2f} | {dist:.2f}m", (0,200,0))

        # ── Fire / smoke (on top) ─────────────────────────────────────────
        for box in fire_results[0].boxes:
            conf = float(box.conf[0])
            if conf < FIRE_CONF:
                continue
            x1,y1,x2,y2 = map(int, box.xyxy[0])
            label = fire_model.names[int(box.cls[0])]
            cx, cy = (x1+x2)//2, (y1+y2)//2
            dist = depth_frame.get_distance(cx, cy)
            is_fire = label.lower() == "fire"

            if is_fire:
                fire_detected  = True
                best_fire_conf = max(best_fire_conf, conf)

            color = (0,0,255) if is_fire else (0,200,255)
            if is_fire:
                draw_corner_brackets(frame, x1, y1, x2, y2, color)
            else:
                cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
            draw_label(frame, x1, y1, f"{label.upper()} {conf:.2f} | {dist:.2f}m", color)

        # ── Alert overlay ─────────────────────────────────────────────────
        if fire_detected:
            pulse = (math.sin(time.monotonic()*6.0)+1)/2
            intensity = pulse * min(1.0, best_fire_conf*1.4)
            draw_pulsing_border(frame, intensity)
            draw_alert_banner(frame, f"FIRE DETECTED  |  conf {best_fire_conf:.2f}", intensity)

        # ── Staggered servo timer (no distance check) ────────────────────
        now = time.monotonic()

        if fire_detected:
            last_fire_time = now
            if fire_start is None:
                fire_start = now
                print("[firebot] Fire seen — timer started", flush=True)

        # Check if fire is still "active" (within grace period)
        fire_active = (last_fire_time is not None and
                       (now - last_fire_time) < GRACE_SEC)

        if fire_active and fire_start is not None:
            elapsed = now - fire_start

            # Servo 1 at 1.3s
            if not servo1_fired and elapsed >= SERVO1_HOLD_SEC:
                servo1_fired = True
                send_cmd("1")
                print("[firebot] >>> SERVO 1 FIRED!", flush=True)

            # Servo 2 at 2.0s
            if not servo2_fired and elapsed >= SERVO2_HOLD_SEC:
                servo2_fired = True
                send_cmd("2")
                print("[firebot] >>> SERVO 2 FIRED!", flush=True)

            # Countdown on screen
            if not servo1_fired:
                remaining = SERVO1_HOLD_SEC - elapsed
                cv2.putText(frame, f"SERVO 1 IN: {remaining:.1f}s", (10, fh-80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            elif not servo2_fired:
                remaining = SERVO2_HOLD_SEC - elapsed
                cv2.putText(frame, f"SERVO 1 FIRED | SERVO 2 IN: {remaining:.1f}s", (10, fh-80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        else:
            # Fire gone for longer than grace period — reset
            if fire_start is not None and not servo1_fired:
                print("[firebot] Fire gone — timer reset", flush=True)
                fire_start = None

        # ── Motor + status ────────────────────────────────────────────────
        both_fired = servo1_fired and servo2_fired

        if both_fired:
            send_cmd("S")
            cv2.putText(frame, "BOTH EXTINGUISHERS ACTIVE", (10, fh-16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        elif servo1_fired:
            send_cmd("S")
            cv2.putText(frame, "EXTINGUISHER 1 ACTIVE — waiting for 2", (10, fh-16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        elif fire_detected:
            send_cmd("S")
            cv2.putText(frame, "MOTOR: STOP (fire detected!)", (10, fh-16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            send_cmd("F")
            cv2.putText(frame, "MOTOR: FORWARD", (10, fh-16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

        cv2.imshow("Firebot", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("\n[firebot] Shutting down...", flush=True)
    pipeline.stop()
    cv2.destroyAllWindows()
    if arduino is not None:
        try:
            arduino.write(b"S")
            arduino.close()
        except Exception:
            pass
    print("[firebot] Done.", flush=True)
