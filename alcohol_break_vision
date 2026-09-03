
"""
ALCOHOL DETECTION
        ↓
ABNORMAL DRIVING VISION
        ↓
ARDUINO + SLOW ROADS BRAKE SYNC

External IP camera → Laptop screen → OpenCV
Arduino → ALCOHOL DETECTED → Camera starts
OpenCV → abnormal driving → TRUE
TRUE → Arduino + SPACEBAR
"""

import cv2
import numpy as np
import time
import threading
import serial
import pytesseract
import pydirectinput

from collections import deque


# ============================================================
# TESSERACT OCR
# ============================================================

pytesseract.pytesseract.tesseract_cmd = (
    r"C:\Program Files\Tesseract-OCR\tesseract.exe"
)


# ============================================================
# CONFIGURATION
# ============================================================

# ---------------- CAMERA ----------------

CAMERA_STREAM_URL = "http://172.16.46.120:8080/video"


# ---------------- ARDUINO ----------------

ARDUINO_PORT = "COM7"
ARDUINO_BAUD = 9600


# ---------------- SPEED ----------------

SPEED_THRESHOLD_KMH = 40.0

CONSECUTIVE_FRAMES_REQUIRED = 5


# ---------------- BRAKE ----------------

BRAKE_CYCLES = 7

BRAKE_ON_SECONDS = 1.0

BRAKE_OFF_SECONDS = 0.5


# ---------------- WARPED SCREEN ----------------

WARPED_WIDTH = 1280
WARPED_HEIGHT = 720


# ---------------- SPEED ROI ----------------

SPEED_ROI = {
    "x1_pct": 0.02,
    "y1_pct": 0.85,
    "x2_pct": 0.16,
    "y2_pct": 0.96,
}


# ---------------- SCREEN DETECTION ----------------

REDETECT_EVERY_N_FRAMES = 15


# ---------------- ZIG-ZAG DETECTION ----------------

ROAD_STRIP_ROI = {
    "y1_pct": 0.55,
    "y2_pct": 0.68,
}

ZIGZAG_WINDOW_FRAMES = 20

ZIGZAG_MIN_DEVIATION_PX = 15

ZIGZAG_DIRECTION_CHANGES_REQUIRED = 4


# ============================================================
# GLOBAL VARIABLES
# ============================================================

running = True

alcohol_detected = False

last_quad = None

frame_counter = 0

consecutive_abnormal_frames = 0

road_center_history = deque(
    maxlen=ZIGZAG_WINDOW_FRAMES
)


# ============================================================
# CONNECT TO ARDUINO
# ============================================================

try:

    arduino = serial.Serial(
        ARDUINO_PORT,
        ARDUINO_BAUD,
        timeout=0.1
    )

    time.sleep(2)

    print("------------------------------------------")
    print("ARDUINO CONNECTED")
    print("------------------------------------------")
    print("Port:", ARDUINO_PORT)
    print("Baud:", ARDUINO_BAUD)
    print()

except serial.SerialException as e:

    print("------------------------------------------")
    print("ERROR: COULD NOT OPEN ARDUINO")
    print("------------------------------------------")
    print(e)

    arduino = None


# ============================================================
# ARDUINO SERIAL READER
# ============================================================

def serial_reader():

    global running
    global alcohol_detected

    while running:

        if arduino is not None:

            try:

                if arduino.in_waiting > 0:

                    line = (
                        arduino.readline()
                        .decode(
                            "utf-8",
                            errors="ignore"
                        )
                        .strip()
                    )

                    if line:

                        print("[ARDUINO]", line)

                        # ------------------------------------
                        # ALCOHOL DETECTED
                        # ------------------------------------

                        if line == "ALCOHOL DETECTED":

                            alcohol_detected = True

            except serial.SerialException:

                print("Arduino serial connection lost.")

                running = False

                break

        time.sleep(0.01)


# ============================================================
# START SERIAL THREAD
# ============================================================

if arduino is not None:

    serial_thread = threading.Thread(
        target=serial_reader,
        daemon=True
    )

    serial_thread.start()


# ============================================================
# ORDER SCREEN CORNERS
# ============================================================

def order_points(points):

    points = points.reshape(4, 2)

    ordered = np.zeros(
        (4, 2),
        dtype="float32"
    )

    # x + y
    sums = points.sum(axis=1)

    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]

    # y - x
    differences = np.diff(
        points,
        axis=1
    )

    ordered[1] = points[
        np.argmin(differences)
    ]

    ordered[3] = points[
        np.argmax(differences)
    ]

    return ordered


# ============================================================
# DETECT LAPTOP SCREEN
# ============================================================

def detect_screen_quad(frame):

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    blurred = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    edges = cv2.Canny(
        blurred,
        50,
        150
    )

    edges = cv2.dilate(
        edges,
        None,
        iterations=2
    )

    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:

        return None

    contours = sorted(
        contours,
        key=cv2.contourArea,
        reverse=True
    )

    frame_area = (
        frame.shape[0] *
        frame.shape[1]
    )

    for contour in contours:

        area = cv2.contourArea(contour)

        # Screen should occupy a reasonable
        # portion of the camera frame

        if area < frame_area * 0.15:

            continue

        perimeter = cv2.arcLength(
            contour,
            True
        )

        approx = cv2.approxPolyDP(
            contour,
            0.02 * perimeter,
            True
        )

        if len(approx) != 4:

            continue

        rectangle = cv2.minAreaRect(
            contour
        )

        width, height = rectangle[1]

        if width == 0 or height == 0:

            continue

        aspect = (
            max(width, height) /
            min(width, height)
        )

        if 1.2 <= aspect <= 2.2:

            return order_points(
                approx.astype("float32")
            )

    return None


# ============================================================
# WARP SCREEN
# ============================================================

def warp_screen(frame, quad):

    destination = np.array(
        [
            [0, 0],

            [
                WARPED_WIDTH - 1,
                0
            ],

            [
                WARPED_WIDTH - 1,
                WARPED_HEIGHT - 1
            ],

            [
                0,
                WARPED_HEIGHT - 1
            ]
        ],
        dtype="float32"
    )

    matrix = cv2.getPerspectiveTransform(
        quad,
        destination
    )

    warped = cv2.warpPerspective(
        frame,
        matrix,
        (
            WARPED_WIDTH,
            WARPED_HEIGHT
        )
    )

    return warped


# ============================================================
# READ SPEED USING TESSERACT
# ============================================================

def read_speed(warped_frame):

    height, width = (
        warped_frame.shape[:2]
    )

    x1 = int(
        SPEED_ROI["x1_pct"] *
        width
    )

    y1 = int(
        SPEED_ROI["y1_pct"] *
        height
    )

    x2 = int(
        SPEED_ROI["x2_pct"] *
        width
    )

    y2 = int(
        SPEED_ROI["y2_pct"] *
        height
    )

    roi = warped_frame[
        y1:y2,
        x1:x2
    ]

    if roi.size == 0:

        return None

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    _, thresholded = cv2.threshold(
        gray,
        150,
        255,
        cv2.THRESH_BINARY
    )

    thresholded = cv2.resize(
        thresholded,
        None,
        fx=3,
        fy=3,
        interpolation=cv2.INTER_CUBIC
    )

    config = (
        "--psm 7 "
        "-c tessedit_char_whitelist=0123456789."
    )

    text = pytesseract.image_to_string(
        thresholded,
        config=config
    ).strip()

    try:

        return float(text)

    except ValueError:

        return None


# ============================================================
# DETECT ROAD CENTER
# ============================================================

def detect_road_center_x(warped_frame):

    height, width = (
        warped_frame.shape[:2]
    )

    y1 = int(
        ROAD_STRIP_ROI["y1_pct"] *
        height
    )

    y2 = int(
        ROAD_STRIP_ROI["y2_pct"] *
        height
    )

    strip = warped_frame[
        y1:y2,
        :
    ]

    if strip.size == 0:

        return None

    hsv = cv2.cvtColor(
        strip,
        cv2.COLOR_BGR2HSV
    )

    # Detect relatively gray road surface

    lower_gray = np.array(
        [0, 0, 40]
    )

    upper_gray = np.array(
        [180, 60, 200]
    )

    mask = cv2.inRange(
        hsv,
        lower_gray,
        upper_gray
    )

    kernel = np.ones(
        (5, 5),
        np.uint8
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:

        return None

    largest = max(
        contours,
        key=cv2.contourArea
    )

    minimum_area = (
        strip.shape[0] *
        strip.shape[1] *
        0.05
    )

    if cv2.contourArea(largest) < minimum_area:

        return None

    moments = cv2.moments(
        largest
    )

    if moments["m00"] == 0:

        return None

    center_x = int(
        moments["m10"] /
        moments["m00"]
    )

    return center_x


# ============================================================
# ZIG-ZAG DETECTION
# ============================================================

def is_zigzagging(history):

    if len(history) < ZIGZAG_WINDOW_FRAMES:

        return False

    mean_center = (
        sum(history) /
        len(history)
    )

    deviations = [
        x - mean_center
        for x in history
    ]

    direction_changes = 0

    last_sign = 0

    for deviation in deviations:

        # Ignore tiny movements

        if abs(deviation) < ZIGZAG_MIN_DEVIATION_PX:

            continue

        sign = (
            1
            if deviation > 0
            else -1
        )

        if (
            last_sign != 0
            and sign != last_sign
        ):

            direction_changes += 1

        last_sign = sign

    return (
        direction_changes >=
        ZIGZAG_DIRECTION_CHANGES_REQUIRED
    )


# ============================================================
# BRAKE SEQUENCE
# ============================================================

def trigger_brake_sequence():

    print()
    print("==========================================")
    print("     ABNORMAL DRIVING DETECTED")
    print("==========================================")

    # --------------------------------------------------------
    # SEND TRUE TO ARDUINO
    # --------------------------------------------------------

    if arduino is not None:

        arduino.write(
            b"TRUE\n"
        )

        arduino.flush()

        print("TRUE SENT TO ARDUINO")

    # --------------------------------------------------------
    # 7 BRAKE CYCLES
    # --------------------------------------------------------

    for i in range(
        1,
        BRAKE_CYCLES + 1
    ):

        print(
            f"BRAKE {i}/{BRAKE_CYCLES}"
        )

        # GAME BRAKE ON

        pydirectinput.keyDown(
            "space"
        )

        time.sleep(
            BRAKE_ON_SECONDS
        )

        # GAME BRAKE OFF

        pydirectinput.keyUp(
            "space"
        )

        time.sleep(
            BRAKE_OFF_SECONDS
        )

    print()
    print("==========================================")
    print("     BRAKE SEQUENCE COMPLETE")
    print("==========================================")
    print()


# ============================================================
# VISION PIPELINE
# ============================================================

def run_vision_pipeline():

    global last_quad
    global frame_counter
    global consecutive_abnormal_frames

    # Reset event variables

    last_quad = None

    frame_counter = 0

    consecutive_abnormal_frames = 0

    road_center_history.clear()

    # --------------------------------------------------------
    # OPEN CAMERA
    # --------------------------------------------------------

    print()
    print("Opening IP camera...")
    print()

    cap = cv2.VideoCapture(
        CAMERA_STREAM_URL
    )

    if not cap.isOpened():

        print(
            "ERROR: Could not open camera:"
        )

        print(
            CAMERA_STREAM_URL
        )

        return

    print(
        ">>> CAMERA PIPELINE ACTIVE <<<"
    )

    try:

        while running:

            ret, frame = cap.read()

            if not ret:

                print(
                    "Frame grab failed..."
                )

                time.sleep(0.1)

                continue

            frame_counter += 1

            # ------------------------------------------------
            # SCREEN DETECTION
            # ------------------------------------------------

            if (
                last_quad is None
                or
                frame_counter %
                REDETECT_EVERY_N_FRAMES == 0
            ):

                detected_quad = (
                    detect_screen_quad(
                        frame
                    )
                )

                if detected_quad is not None:

                    last_quad = detected_quad

            # ------------------------------------------------
            # NO SCREEN FOUND
            # ------------------------------------------------

            if last_quad is None:

                cv2.imshow(
                    "Camera - Screen Not Found",
                    frame
                )

                if (
                    cv2.waitKey(1) &
                    0xFF
                ) == ord("q"):

                    break

                continue

            # ------------------------------------------------
            # WARP SCREEN
            # ------------------------------------------------

            warped = warp_screen(
                frame,
                last_quad
            )

            # ------------------------------------------------
            # SPEED
            # ------------------------------------------------

            speed = read_speed(
                warped
            )

            # ------------------------------------------------
            # ROAD CENTER
            # ------------------------------------------------

            road_center = (
                detect_road_center_x(
                    warped
                )
            )

            zigzag_flagged = False

            if road_center is not None:

                road_center_history.append(
                    road_center
                )

                zigzag_flagged = (
                    is_zigzagging(
                        road_center_history
                    )
                )

            # ------------------------------------------------
            # SPEED ANALYSIS
            # ------------------------------------------------

            speed_flagged = False

            if speed is not None:

                print(
                    f"Speed: {speed:.1f} km/h"
                )

                if (
                    speed >
                    SPEED_THRESHOLD_KMH
                ):

                    consecutive_abnormal_frames += 1

                else:

                    consecutive_abnormal_frames = 0

                if (
                    consecutive_abnormal_frames >=
                    CONSECUTIVE_FRAMES_REQUIRED
                ):

                    speed_flagged = True

            else:

                print(
                    "Speed: OCR failed"
                )

            # ------------------------------------------------
            # FINAL ABNORMALITY DECISION
            # ------------------------------------------------

            if (
                speed_flagged
                or
                zigzag_flagged
            ):

                if (
                    speed_flagged
                    and
                    zigzag_flagged
                ):

                    reason = (
                        "SPEED + ZIG-ZAG"
                    )

                elif speed_flagged:

                    reason = "SPEED"

                else:

                    reason = "ZIG-ZAG"

                print()
                print(
                    "ABNORMALITY DETECTED"
                )

                print(
                    "Reason:",
                    reason
                )

                # ------------------------------------------------
                # BRAKE
                # ------------------------------------------------

                trigger_brake_sequence()

                # Stop processing this alcohol event

                break

            # ------------------------------------------------
            # DEBUG DISPLAY
            # ------------------------------------------------

            debug_frame = frame.copy()

            cv2.polylines(
                debug_frame,
                [
                    last_quad.astype(
                        int
                    )
                ],
                True,
                (0, 255, 0),
                2
            )

            cv2.imshow(
                "Detected Laptop Screen",
                debug_frame
            )

            cv2.imshow(
                "Slow Roads - Warped",
                warped
            )

            # ------------------------------------------------
            # QUIT
            # ------------------------------------------------

            if (
                cv2.waitKey(1) &
                0xFF
            ) == ord("q"):

                break

    finally:

        cap.release()

        cv2.destroyAllWindows()

        print()
        print(
            ">>> CAMERA PIPELINE CLOSED <<<"
        )
        print()


# ============================================================
# MAIN SYSTEM
# ============================================================

def main():

    global running
    global alcohol_detected

    print()
    print("==========================================")
    print(" ALCOHOL + ABNORMAL DRIVING SYSTEM")
    print("==========================================")
    print()

    print(
        "Waiting for ALCOHOL DETECTED..."
    )

    print(
        "Camera is currently CLOSED."
    )

    print()

    try:

        while running:

            # ------------------------------------------------
            # WAIT FOR ALCOHOL
            # ------------------------------------------------

            if alcohol_detected:

                # Consume the alcohol event

                alcohol_detected = False

                # Start vision system

                run_vision_pipeline()

                print(
                    "Returning to IDLE..."
                )

                print(
                    "Waiting for next ALCOHOL DETECTED..."
                )

                print()

            time.sleep(0.05)

    except KeyboardInterrupt:

        print()
        print(
            "Stopping system..."
        )

    finally:

        running = False

        # Release Spacebar just in case

        try:

            pydirectinput.keyUp(
                "space"
            )

        except Exception:

            pass

        # Close Arduino

        if (
            arduino is not None
            and
            arduino.is_open
        ):

            arduino.close()

        cv2.destroyAllWindows()

        print(
            "System stopped."
        )


# ============================================================
# START
# ============================================================

if __name__ == "__main__":

    main()

