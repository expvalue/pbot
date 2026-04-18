import math
import os
import time
import urllib.request

import cv2
import numpy as np
import pyrealsense2 as rs
import serial
from serial.tools import list_ports
from ultralytics import YOLO

WEIGHTS_URL = "https://github.com/luminous0219/fire-and-smoke-detection-yolov8/raw/main/weights/best.pt"
WEIGHTS_PATH = "fire_yolov8n.pt"

CONF_THRESHOLD = 0.35
MODEL_IMGSZ = 320
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
INFERENCE_EVERY_N_FRAMES = 3
WINDOW_NAME = "Fire Detection"

SERIAL_BAUD = 9600
SERIAL_PORT = os.getenv("PBOT_SERIAL_PORT", "").strip()
SERIAL_READY_DELAY_SECONDS = 2.0
COMMAND_HEARTBEAT_SECONDS = 0.15

TARGET_DISTANCE_M = 0.50
STOP_DISTANCE_TOLERANCE_M = 0.03
FRAME_CENTER_X = CAMERA_WIDTH // 2
CENTER_TOLERANCE_PX = 55


def ensure_weights():
    if not os.path.exists(WEIGHTS_PATH):
        print(f"Downloading fire/smoke weights to {WEIGHTS_PATH}...")
        urllib.request.urlretrieve(WEIGHTS_URL, WEIGHTS_PATH)
        print("Done.")


def auto_detect_serial_port():
    if SERIAL_PORT:
        return SERIAL_PORT

    ports = list(list_ports.comports())
    if not ports:
        return None

    preferred = []
    for port in ports:
        text = " ".join(
            part for part in [port.device, port.description, port.manufacturer] if part
        ).lower()
        if any(token in text for token in ("arduino", "mega", "ch340", "usb serial")):
            preferred.append(port.device)

    if preferred:
        return preferred[0]

    return ports[0].device


def connect_arduino():
    port = auto_detect_serial_port()
    if not port:
        print("Arduino serial: no COM port found, running camera-only mode.")
        return None

    try:
        arduino = serial.Serial(port, SERIAL_BAUD, timeout=0.05, write_timeout=0.2)
        time.sleep(SERIAL_READY_DELAY_SECONDS)
        arduino.reset_input_buffer()
        print(f"Arduino serial connected on {port} @ {SERIAL_BAUD}.")
        return arduino
    except Exception as exc:
        print(f"Arduino serial unavailable on {port}: {exc}")
        print("Running camera-only mode.")
        return None


def send_arduino_command(arduino, command):
    if arduino is None:
        return

    try:
        arduino.write(f"{command}\n".encode("ascii"))
        print(f"[Arduino] {command}")
    except Exception as exc:
        print(f"Failed to send '{command}' to Arduino: {exc}")


def drain_arduino_serial(arduino):
    if arduino is None:
        return

    try:
        while arduino.in_waiting > 0:
            msg = arduino.readline().decode(errors="ignore").strip()
            if msg:
                print(f"[Arduino->PC] {msg}")
    except Exception:
        pass


def safe_stop_pipeline(pipeline):
    if pipeline is None:
        return
    try:
        pipeline.stop()
    except BaseException:
        pass


def safe_close_serial(arduino):
    if arduino is None:
        return
    try:
        if arduino.is_open:
            arduino.close()
    except BaseException:
        pass


def get_box_distance_m(depth_frame, x1, y1, x2, y2):
    width = depth_frame.get_width()
    height = depth_frame.get_height()

    cx = min(max((x1 + x2) // 2, 0), width - 1)
    cy = min(max((y1 + y2) // 2, 0), height - 1)
    sample_radius = max(2, min(8, min(x2 - x1, y2 - y1) // 8))

    distances = []
    for py in range(max(0, cy - sample_radius), min(height, cy + sample_radius + 1)):
        for px in range(max(0, cx - sample_radius), min(width, cx + sample_radius + 1)):
            distance = depth_frame.get_distance(px, py)
            if 0.05 <= distance <= 8.0:
                distances.append(distance)

    if distances:
        return float(np.median(distances))

    return float(depth_frame.get_distance(cx, cy))


def draw_corner_brackets(img, x1, y1, x2, y2, color, thickness=3, length=20):
    cv2.line(img, (x1, y1), (x1 + length, y1), color, thickness)
    cv2.line(img, (x1, y1), (x1, y1 + length), color, thickness)
    cv2.line(img, (x2, y1), (x2 - length, y1), color, thickness)
    cv2.line(img, (x2, y1), (x2, y1 + length), color, thickness)
    cv2.line(img, (x1, y2), (x1 + length, y2), color, thickness)
    cv2.line(img, (x1, y2), (x1, y2 - length), color, thickness)
    cv2.line(img, (x2, y2), (x2 - length, y2), color, thickness)
    cv2.line(img, (x2, y2), (x2, y2 - length), color, thickness)


def draw_pulsing_border(img, intensity):
    h, w = img.shape[:2]
    thickness = int(6 + 10 * intensity)
    red = (0, 0, int(180 + 75 * intensity))
    cv2.rectangle(img, (0, 0), (w - 1, h - 1), red, thickness)


def draw_alert_banner(img, text, intensity):
    h, w = img.shape[:2]
    banner_h = 48
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, banner_h), (0, 0, 255), -1)
    alpha = 0.55 + 0.35 * intensity
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    tx = (w - tw) // 2
    ty = (banner_h + th) // 2 - 2
    cv2.putText(img, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)


def draw_label(img, x1, y1, text, color):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    top = max(0, y1 - th - 8)
    cv2.rectangle(img, (x1, top), (x1 + tw + 6, y1), color, -1)
    cv2.putText(img, text, (x1 + 3, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)


def draw_crosshair(img, cx, cy, color):
    cv2.line(img, (cx - 12, cy), (cx + 12, cy), color, 2)
    cv2.line(img, (cx, cy - 12), (cx, cy + 12), color, 2)
    cv2.circle(img, (cx, cy), 4, color, -1)


def run():
    ensure_weights()

    model = YOLO(WEIGHTS_PATH)
    arduino = connect_arduino()
    send_arduino_command(arduino, "FIRE_STOP")
    send_arduino_command(arduino, "FIRE_OFF")

    print("Classes:", model.names)
    print(
        f"Targeting fire at {TARGET_DISTANCE_M:.2f} m, "
        f"YOLO imgsz={MODEL_IMGSZ}, infer every {INFERENCE_EVERY_N_FRAMES} frame(s)."
    )
    print("Press 'q' in the video window to quit.")
    print("Starting RealSense camera...")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, CAMERA_FPS)
    config.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, CAMERA_FPS)
    try:
        pipeline.start(config)
    except RuntimeError as exc:
        send_arduino_command(arduino, "FIRE_STOP")
        send_arduino_command(arduino, "FIRE_OFF")
        safe_close_serial(arduino)
        raise RuntimeError(
            "RealSense camera could not start. Make sure the camera is plugged in, "
            "powered, and not in use by another app."
        ) from exc
    align = rs.align(rs.stream.color)
    print("RealSense camera ready.")

    cached_detections = []
    frame_index = 0
    last_infer_ms = 0.0
    last_drive_command = ""
    last_drive_sent_at = 0.0
    servo_active = False
    state = "SEARCHING"

    def send_drive_command(command, force=False):
        nonlocal last_drive_command, last_drive_sent_at
        now = time.monotonic()
        if force or command != last_drive_command or (now - last_drive_sent_at) >= COMMAND_HEARTBEAT_SECONDS:
            send_arduino_command(arduino, command)
            last_drive_command = command
            last_drive_sent_at = now

    try:
        while True:
            drain_arduino_serial(arduino)

            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            frame_index += 1

            if frame_index % INFERENCE_EVERY_N_FRAMES == 1:
                infer_start = time.perf_counter()
                results = model.predict(
                    source=frame,
                    verbose=False,
                    imgsz=MODEL_IMGSZ,
                    conf=CONF_THRESHOLD,
                    max_det=10,
                )
                last_infer_ms = (time.perf_counter() - infer_start) * 1000.0
                cached_detections = []

                for box in results[0].boxes:
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = model.names[int(box.cls[0])]
                    distance = get_box_distance_m(depth_frame, x1, y1, x2, y2)
                    cached_detections.append((x1, y1, x2, y2, label, conf, distance))

            best_fire_target = None
            best_fire_conf = 0.0

            for x1, y1, x2, y2, label, conf, distance in cached_detections:
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                is_fire = label.lower() == "fire"

                color = (0, 0, 255) if is_fire else (0, 200, 255)
                if is_fire:
                    draw_corner_brackets(frame, x1, y1, x2, y2, color, thickness=3, length=22)
                    draw_crosshair(frame, cx, cy, color)
                    if conf > best_fire_conf:
                        best_fire_conf = conf
                        best_fire_target = (x1, y1, x2, y2, cx, cy, distance)
                else:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                distance_text = f"{distance:.2f}m" if distance > 0 else "---"
                draw_label(frame, x1, y1, f"{label.upper()} {conf:.2f} | {distance_text}", color)

            desired_drive_command = "FIRE_STOP"
            desired_servo_active = False
            distance_text = "---"
            state_color = (255, 255, 255)

            if best_fire_target is None:
                state = "SEARCHING"
                state_color = (0, 200, 255)
            else:
                _, _, _, _, fire_cx, _, fire_distance = best_fire_target
                offset = fire_cx - FRAME_CENTER_X
                distance_text = f"{fire_distance:.2f}m" if fire_distance > 0 else "---"

                if fire_distance <= 0:
                    state = "WAITING_DEPTH"
                    state_color = (0, 200, 255)
                elif abs(offset) > CENTER_TOLERANCE_PX:
                    if offset < 0:
                        state = "ALIGN_LEFT"
                        desired_drive_command = "FIRE_LEFT"
                    else:
                        state = "ALIGN_RIGHT"
                        desired_drive_command = "FIRE_RIGHT"
                    state_color = (0, 165, 255)
                elif fire_distance > TARGET_DISTANCE_M + STOP_DISTANCE_TOLERANCE_M:
                    state = "APPROACHING"
                    desired_drive_command = "FIRE_AHEAD"
                    state_color = (0, 255, 0)
                else:
                    state = "AT_TARGET"
                    desired_drive_command = "FIRE_STOP"
                    desired_servo_active = True
                    state_color = (0, 0, 255)

            send_drive_command(desired_drive_command)

            if desired_servo_active and not servo_active:
                send_arduino_command(arduino, "FIRE_ON")
                servo_active = True
            elif not desired_servo_active and servo_active:
                send_arduino_command(arduino, "FIRE_OFF")
                servo_active = False

            if best_fire_target is not None:
                pulse = (math.sin(time.monotonic() * 6.0) + 1) / 2
                intensity = pulse * min(1.0, max(best_fire_conf, 0.4) * 1.2)
                draw_pulsing_border(frame, intensity)
                draw_alert_banner(
                    frame,
                    f"FIRE  dist:{distance_text}  target:{TARGET_DISTANCE_M:.2f}m  state:{state}",
                    intensity,
                )

            cv2.line(frame, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, CAMERA_HEIGHT), (80, 80, 80), 1)

            serial_text = "Arduino: disconnected"
            if arduino is not None:
                serial_text = f"Arduino: {arduino.port} | drive: {desired_drive_command} | servo: {'ON' if servo_active else 'OFF'}"

            status = f"Infer: {last_infer_ms:.0f} ms | state: {state} | fire dist: {distance_text}"
            cv2.putText(frame, status, (10, CAMERA_HEIGHT - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.55, state_color, 2)
            cv2.putText(frame, serial_text, (10, CAMERA_HEIGHT - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("Stopping on Ctrl+C...")
    finally:
        send_drive_command("FIRE_STOP", force=True)
        send_arduino_command(arduino, "FIRE_OFF")
        safe_stop_pipeline(pipeline)
        safe_close_serial(arduino)
        try:
            cv2.destroyAllWindows()
        except BaseException:
            pass


if __name__ == "__main__":
    run()
