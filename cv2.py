import cv2
import time
import csv
import os
import argparse
import numpy as np
from ultralytics import YOLO
from collections import defaultdict

# ---------------------------------
# CONFIGURATION
# ---------------------------------
MODEL_PATH = "yolo11n.pt"
DEFAULT_SPEED_LIMIT = 60  # km/h

# Calibrate these values for your road.
# Example: two reference lines 20 meters apart.
ROAD_DISTANCE_METERS = 20.0

# Set to True only after calibration.
CALIBRATION_ENABLED = False

# ---------------------------------
# ARGUMENTS
# ---------------------------------
parser = argparse.ArgumentParser(
    description="Vehicle Detection, Tracking and Speed Monitoring"
)
parser.add_argument(
    "--source", default="0",
    help="0 for webcam or path to a video file"
)
parser.add_argument(
    "--speed-limit", type=float, default=DEFAULT_SPEED_LIMIT
)
parser.add_argument(
    "--output", default="vehicle_log.csv"
)
args = parser.parse_args()

source = int(args.source) if args.source.isdigit() else args.source
speed_limit = args.speed_limit

# ---------------------------------
# LOAD MODEL
# ---------------------------------
model = YOLO(MODEL_PATH)

# COCO classes: car=2, motorcycle=3, bus=5, truck=7
VEHICLE_CLASSES = [2, 3, 5, 7]

# ---------------------------------
# VIDEO INPUT
# ---------------------------------
cap = cv2.VideoCapture(source)

if not cap.isOpened():
    raise RuntimeError(f"Cannot open video source: {source}")

fps = cap.get(cv2.CAP_PROP_FPS)
if not fps or fps <= 0:
    fps = 30.0

# ---------------------------------
# TRACKING AND SPEED DATA
# ---------------------------------
previous_positions = {}
previous_times = {}
speed_history = defaultdict(list)
vehicle_classes = {}
logged_vehicles = set()

# For calibrated line-to-line speed measurement
# Define line positions as fractions of frame height.
line1_fraction = 0.40
line2_fraction = 0.70
line1_times = {}
line2_times = {}
frame_height = None

# CSV OUTPUT
file_exists = os.path.exists(args.output)
csv_file = open(args.output, "a", newline="")
csv_writer = csv.writer(csv_file)

if not file_exists or os.path.getsize(args.output) == 0:
    csv_writer.writerow([
        "timestamp", "vehicle_id", "vehicle_type",
        "speed_kmh", "speed_limit_kmh", "status"
    ])

print("Vehicle monitoring started. Press Q to quit.")

# ---------------------------------
# MAIN LOOP
# ---------------------------------
try:
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        height, width = frame.shape[:2]

        if frame_height is None:
            frame_height = height

        line1_y = int(height * line1_fraction)
        line2_y = int(height * line2_fraction)

        # YOLO detection + ByteTrack
        results = model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            classes=VEHICLE_CLASSES,
            conf=0.35,
            verbose=False
        )

        current_time = time.time()
        result = results[0]

        # Draw reference lines
        cv2.line(frame, (0, line1_y), (width, line1_y),
                 (255, 255, 0), 2)
        cv2.line(frame, (0, line2_y), (width, line2_y),
                 (255, 255, 0), 2)

        cv2.putText(
            frame, f"Speed limit: {speed_limit:.0f} km/h",
            (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (255, 255, 255), 2
        )

        if result.boxes is not None and result.boxes.id is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            ids = result.boxes.id.int().cpu().tolist()
            classes = result.boxes.cls.int().cpu().tolist()

            for box, track_id, class_id in zip(boxes, ids, classes):
                x1, y1, x2, y2 = map(int, box)

                # Bottom-center point approximates road contact.
                cx = (x1 + x2) // 2
                cy = y2

                vehicle_name = model.names[class_id]
                vehicle_classes[track_id] = vehicle_name

                # Estimate speed from displacement.
                # This pixel-based estimate is NOT a real-world speed
                # unless a valid road calibration is supplied.
                speed = 0.0

                if track_id in previous_positions:
                    old_x, old_y = previous_positions[track_id]
                    dt = current_time - previous_times[track_id]

                    if dt > 0:
                        pixel_distance = np.hypot(
                            cx - old_x, cy - old_y
                        )

                        # Only calculate km/h when calibrated.
                        # A fixed pixels-per-meter conversion is
                        # not valid across arbitrary perspective.
                        if CALIBRATION_ENABLED:
                            pixels_per_meter = 10.0  # Replace!
                            distance_m = pixel_distance / pixels_per_meter
                            speed = (distance_m / dt) * 3.6

                        speed_history[track_id].append(speed)

                previous_positions[track_id] = (cx, cy)
                previous_times[track_id] = current_time

                # Reference-line crossing timestamps.
                if cy >= line1_y and track_id not in line1_times:
                    line1_times[track_id] = current_time

                if (
                    cy >= line2_y
                    and track_id in line1_times
                    and track_id not in line2_times
                ):
                    line2_times[track_id] = current_time

                # Calibrated line-to-line speed:
                # assumes lines correspond to known road distance.
                if (
                    CALIBRATION_ENABLED
                    and track_id in line1_times
                    and track_id in line2_times
                ):
                    elapsed = (
                        line2_times[track_id] - line1_times[track_id]
                    )

                    if elapsed > 0:
                        speed = (
                            ROAD_DISTANCE_METERS / elapsed
                        ) * 3.6

                # Smooth instantaneous speed estimates.
                recent_speeds = speed_history[track_id][-5:]
                if recent_speeds:
                    display_speed = float(np.mean(recent_speeds))
                else:
                    display_speed = speed

                # Speed measurement is meaningful only when calibrated.
                is_overspeed = (
                    CALIBRATION_ENABLED
                    and display_speed > speed_limit
                )

                color = (0, 0, 255) if is_overspeed else (0, 255, 0)

                cv2.rectangle(
                    frame, (x1, y1), (x2, y2), color, 2
                )

                if CALIBRATION_ENABLED:
                    label = (
                        f"{vehicle_name} ID:{track_id} "
                        f"{display_speed:.1f} km/h"
                    )
                else:
                    label = (
                        f"{vehicle_name} ID:{track_id} "
                        "Speed: calibrate road"
                    )

                if is_overspeed:
                    label += " OVERSPEED"

                cv2.putText(
                    frame, label,
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 2
                )

                # Log each tracked vehicle once, after crossing
                # both lines and obtaining a calibrated speed.
                if (
                    CALIBRATION_ENABLED
                    and track_id in line1_times
                    and track_id in line2_times
                    and track_id not in logged_vehicles
                ):
                    elapsed = (
                        line2_times[track_id] - line1_times[track_id]
                    )

                    if elapsed > 0:
                        measured_speed = (
                            ROAD_DISTANCE_METERS / elapsed
                        ) * 3.6

                        status = (
                            "OVERSPEED"
                            if measured_speed > speed_limit
                            else "NORMAL"
                        )

                        csv_writer.writerow([
                            time.strftime("%Y-%m-%d %H:%M:%S"),
                            track_id,
                            vehicle_name,
                            round(measured_speed, 2),
                            speed_limit,
                            status
                        ])
                        csv_file.flush()
                        logged_vehicles.add(track_id)

        cv2.imshow(
            "Real-Time Vehicle Detection and Speed Monitoring",
            frame
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    cap.release()
    csv_file.close()
    cv2.destroyAllWindows()