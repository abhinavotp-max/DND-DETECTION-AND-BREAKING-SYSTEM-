"""
LOW-LATENCY ALCOHOL + RASH DRIVING SYSTEM
=========================================

Arduino
   ↓
MQ-3 detects alcohol
   ↓
Arduino sends "ALCOHOL DETECTED"
   ↓
Python starts MSS SCREEN CAPTURE
   ↓
OpenCV analyzes Slow Roads directly
   ↓
Rapid / aggressive weaving detected
   ↓
TRUE → Arduino
   +
SPACEBAR BRAKING
   ↓
7 brake cycles
   ↓
SPACEBAR HELD during final stop / beep
   ↓
SPACEBAR RELEASED
   ↓
Return to IDLE

IMPORTANT:
This is a driving-game prototype. The Spacebar represents
the simulated brake in Slow Roads.
"""

import cv2
import numpy as np
import time
import threading
import serial
import pytesseract
import pydirectinput
import mss

from collections import deque


# ============================================================
# TESSERACT
# ============================================================

pytesseract.pytesseract.tesseract_cmd = (
    r"C:\Program Files\Tesseract-OCR\tesseract.exe"
)


# ============================================================
# ARDUINO
# ============================================================

ARDUINO_PORT = "COM7"
ARDUINO_BAUD = 9600


# ============================================================
# SPEED SETTINGS
# ============================================================

SPEED_THRESHOLD_KMH = 40.0

# OCR is secondary now
OCR_EVERY_N_FRAMES = 12

# Require 3 NEW OCR readings above threshold
SPEED_CONSECUTIVE_READINGS = 3


# ============================================================
# BRAKE SETTINGS
# ============================================================

BRAKE_CYCLES = 7

BRAKE_ON_SECONDS = 1.0
BRAKE_OFF_SECONDS = 0.5

FINAL_STOP_SECONDS = 5.0


# ============================================================
# MSS SCREEN CAPTURE
# ============================================================

# 0 = primary monitor
MONITOR_NUMBER = 1


# ============================================================
# RASH DRIVING DETECTION
# ============================================================

# The road center is measured as a percentage of screen width.

# How much the vehicle/road center has to move before
# movement is considered significant.
LATERAL_MOVEMENT_THRESHOLD_PX = 12

# Minimum total left/right movement needed.
TOTAL_SWING_THRESHOLD_PX = 30

# Number of direction reversals required.
DIRECTION_CHANGES_REQUIRED = 3

# Number of recent frames considered.
ZIGZAG_WINDOW_FRAMES = 18

# Sudden movement threshold.
# Lower = more sensitive.
SUDDEN_MOVEMENT_THRESHOLD_PX = 9


# ============================================================
# ROAD REGION
# ============================================================

# These are percentages of the FULL SCREEN.

ROAD_Y1_PERCENT = 0.52
ROAD_Y2_PERCENT = 0.72


# ============================================================
# SPEED REGION
# ============================================================

# Slow Roads speedometer is normally near the bottom-left.

SPEED_X1_PERCENT = 0.02
SPEED_Y1_PERCENT = 0.85
SPEED_X2_PERCENT = 0.16
SPEED_Y2_PERCENT = 0.96


# ============================================================
# GLOBAL STATE
# ============================================================

running = True
alcohol_detected = False

frame_counter = 0

speed_last_value = None
speed_consecutive_readings = 0

road_center_history = deque(
    maxlen=ZIGZAG_WINDOW_FRAMES
)


# ============================================================
# ARDUINO CONNECTION
# ============================================================

try:

    arduino = serial.Serial(
        ARDUINO_PORT,
        ARDUINO_BAUD,
        timeout=0.05
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

                if arduino.in_waiting:

                    line = (
                        arduino.readline()
                        .decode(
                            "utf-8",
                            errors="ignore"
                        )
                        .strip()
                    )

                    if line:

                        print(
                            "[ARDUINO]",
                            line
                        )

                        if line == "ALCOHOL DETECTED":

                            alcohol_detected = True

            except serial.SerialException:

                print(
                    "Arduino serial connection lost."
                )

                running = False

                break

        time.sleep(0.005)


if arduino is not None:

    threading.Thread(
        target=serial_reader,
        daemon=True
    ).start()


# ============================================================
# MSS SCREEN CAPTURE
# ============================================================

class MSSCapture:

    def __init__(self):

        self.sct = mss.mss()

        monitor_index = MONITOR_NUMBER

        if (
            monitor_index < 1
            or
            monitor_index >= len(
                self.sct.monitors
            )
        ):

            monitor_index = 1

        self.monitor = self.sct.monitors[
            monitor_index
        ]

        self.running = True

        self.frame = None

        self.lock = threading.Lock()

        self.thread = threading.Thread(
            target=self.capture_loop,
            daemon=True
        )

        self.thread.start()


    def capture_loop(self):

        while self.running:

            try:

                screenshot = self.sct.grab(
                    self.monitor
                )

                frame = np.array(
                    screenshot
                )

                # BGRA → BGR
                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGRA2BGR
                )

                with self.lock:

                    # Always overwrite old frame.
                    # Never build up a frame queue.
                    self.frame = frame

            except Exception as e:

                print(
                    "MSS capture error:",
                    e
                )

                time.sleep(0.01)


    def read(self):

        with self.lock:

            if self.frame is None:

                return False, None

            return (
                True,
                self.frame.copy()
            )


    def release(self):

        self.running = False

        if self.thread.is_alive():

            self.thread.join(
                timeout=0.5
            )

        try:

            self.sct.close()

        except:

            pass


# ============================================================
# ROAD CENTER DETECTION
# ============================================================

def detect_road_center_x(frame):

    height, width = frame.shape[:2]

    y1 = int(
        ROAD_Y1_PERCENT *
        height
    )

    y2 = int(
        ROAD_Y2_PERCENT *
        height
    )

    strip = frame[
        y1:y2,
        :
    ]

    if strip.size == 0:

        return None


    # Convert to HSV
    hsv = cv2.cvtColor(
        strip,
        cv2.COLOR_BGR2HSV
    )


    # Detect relatively gray road surface
    lower_gray = np.array(
        [0, 0, 35]
    )

    upper_gray = np.array(
        [180, 75, 210]
    )


    mask = cv2.inRange(
        hsv,
        lower_gray,
        upper_gray
    )


    # Small kernel = faster and more responsive
    kernel = np.ones(
        (3, 3),
        np.uint8
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )


    # Remove tiny objects
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
        0.03
    )


    if cv2.contourArea(
        largest
    ) < minimum_area:

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
# SENSITIVE RASH / ZIG-ZAG DETECTOR
# ============================================================

def detect_rash_driving(history):

    if len(history) < 6:

        return False, 0


    values = list(history)


    # --------------------------------------------------------
    # 1. TOTAL LATERAL SWING
    # --------------------------------------------------------

    total_range = (
        max(values) -
        min(values)
    )


    # --------------------------------------------------------
    # 2. DIRECTION CHANGES
    # --------------------------------------------------------

    direction_changes = 0

    last_direction = 0


    for i in range(
        1,
        len(values)
    ):

        movement = (
            values[i] -
            values[i - 1]
        )


        if abs(movement) < (
            LATERAL_MOVEMENT_THRESHOLD_PX
        ):

            continue


        direction = (
            1
            if movement > 0
            else -1
        )


        if (
            last_direction != 0
            and
            direction != last_direction
        ):

            direction_changes += 1


        last_direction = direction


    # --------------------------------------------------------
    # 3. SUDDEN MOVEMENT
    # --------------------------------------------------------

    sudden_movement = False


    for i in range(
        1,
        len(values)
    ):

        movement = abs(
            values[i] -
            values[i - 1]
        )


        if movement >= (
            SUDDEN_MOVEMENT_THRESHOLD_PX
        ):

            sudden_movement = True

            break


    # --------------------------------------------------------
    # DECISION
    # --------------------------------------------------------

    # Strongest condition:
    # significant swing + repeated reversal

    if (
        total_range >=
        TOTAL_SWING_THRESHOLD_PX
        and
        direction_changes >=
        DIRECTION_CHANGES_REQUIRED
    ):

        return True, direction_changes


    # Extremely sudden movement + repeated reversal

    if (
        sudden_movement
        and
        direction_changes >= 2
        and
        total_range >= 24
    ):

        return True, direction_changes


    return False, direction_changes


# ============================================================
# SPEED OCR
# ============================================================

def read_speed(frame):

    height, width = frame.shape[:2]


    x1 = int(
        SPEED_X1_PERCENT *
        width
    )

    y1 = int(
        SPEED_Y1_PERCENT *
        height
    )

    x2 = int(
        SPEED_X2_PERCENT *
        width
    )

    y2 = int(
        SPEED_Y2_PERCENT *
        height
    )


    roi = frame[
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
        fx=2,
        fy=2,
        interpolation=cv2.INTER_LINEAR
    )


    config = (
        "--psm 7 "
        "-c tessedit_char_whitelist=0123456789."
    )


    try:

        text = pytesseract.image_to_string(
            thresholded,
            config=config
        ).strip()


        text = text.replace(
            " ",
            ""
        )


        return float(text)


    except:

        return None


# ============================================================
# BRAKE SEQUENCE
# ============================================================

def trigger_brake_sequence():

    print()
    print(
        "##########################################"
    )
    print(
        "       ABNORMAL DRIVING DETECTED"
    )
    print(
        "##########################################"
    )


    # --------------------------------------------------------
    # SEND TRUE TO ARDUINO
    # --------------------------------------------------------

    if arduino is not None:

        try:

            arduino.write(
                b"TRUE\n"
            )

            arduino.flush()

            print(
                "TRUE SENT TO ARDUINO"
            )

        except Exception as e:

            print(
                "Arduino send error:",
                e
            )


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


        pydirectinput.keyDown(
            "space"
        )


        time.sleep(
            BRAKE_ON_SECONDS
        )


        pydirectinput.keyUp(
            "space"
        )


        time.sleep(
            BRAKE_OFF_SECONDS
        )


    # --------------------------------------------------------
    # FINAL STOP
    #
    # IMPORTANT:
    # HOLD SPACEBAR FOR THE ENTIRE FINAL
    # 5 SECOND STOP/BEEP PERIOD.
    # --------------------------------------------------------

    print()
    print(
        "=========================================="
    )

    print(
        "CAR STOPPING"
    )

    print(
        "SPACEBAR HELD"
    )

    print(
        "=========================================="
    )


    pydirectinput.keyDown(
        "space"
    )


    # Same duration as Arduino's final buzzer
    time.sleep(
        FINAL_STOP_SECONDS
    )


    pydirectinput.keyUp(
        "space"
    )


    print(
        "SPACEBAR RELEASED"
    )


    print()
    print(
        "=========================================="
    )

    print(
        "BRAKE SEQUENCE COMPLETE"
    )

    print(
        "=========================================="
    )

    print()


# ============================================================
# VISION PIPELINE
# ============================================================

def run_vision_pipeline():

    global frame_counter
    global speed_last_value
    global speed_consecutive_readings


    frame_counter = 0

    speed_last_value = None

    speed_consecutive_readings = 0


    road_center_history.clear()


    print()
    print(
        "=========================================="
    )

    print(
        "STARTING MSS SCREEN CAPTURE"
    )

    print(
        "=========================================="
    )

    print()


    cap = MSSCapture()


    # Wait for first frame

    start_wait = time.time()


    while (
        cap.frame is None
        and
        time.time() -
        start_wait < 3
    ):

        time.sleep(
            0.005
        )


    if cap.frame is None:

        print(
            "ERROR: MSS DID NOT RETURN A FRAME"
        )

        cap.release()

        return


    print(
        ">>> MSS LOW-LATENCY CAPTURE ACTIVE <<<"
    )

    print()


    try:

        while running:

            ret, frame = cap.read()


            if not ret:

                time.sleep(
                    0.005
                )

                continue


            frame_counter += 1


            # =================================================
            # ROAD DETECTION — EVERY FRAME
            # =================================================

            road_center = (
                detect_road_center_x(
                    frame
                )
            )


            rash_flagged = False

            direction_changes = 0


            if road_center is not None:

                road_center_history.append(
                    road_center
                )


                (
                    rash_flagged,
                    direction_changes
                ) = detect_rash_driving(
                    road_center_history
                )


            # =================================================
            # SPEED OCR — ONLY EVERY N FRAMES
            # =================================================

            new_speed_reading = False


            if (
                frame_counter %
                OCR_EVERY_N_FRAMES == 0
            ):

                speed = read_speed(
                    frame
                )


                if speed is not None:

                    speed_last_value = speed

                    new_speed_reading = True

                    print(
                        f"Speed: {speed:.1f} km/h"
                    )


            # =================================================
            # SPEED DECISION
            # =================================================

            speed_flagged = False


            # IMPORTANT:
            # Only increment the counter when a NEW OCR
            # reading arrives. This avoids counting the
            # same OCR value repeatedly.

            if new_speed_reading:

                if (
                    speed_last_value >
                    SPEED_THRESHOLD_KMH
                ):

                    speed_consecutive_readings += 1

                else:

                    speed_consecutive_readings = 0


                if (
                    speed_consecutive_readings
                    >=
                    SPEED_CONSECUTIVE_READINGS
                ):

                    speed_flagged = True


            # =================================================
            # ABNORMAL DRIVING DECISION
            # =================================================

            if (
                rash_flagged
                or
                speed_flagged
            ):


                if (
                    rash_flagged
                    and
                    speed_flagged
                ):

                    reason = (
                        "RASH DRIVING + SPEED"
                    )


                elif rash_flagged:

                    reason = (
                        "AGGRESSIVE WEAVING"
                    )


                else:

                    reason = (
                        "EXCESSIVE SPEED"
                    )


                print()
                print(
                    "##########################################"
                )

                print(
                    "ABNORMALITY DETECTED"
                )

                print(
                    "Reason:",
                    reason
                )

                print(
                    "Direction changes:",
                    direction_changes
                )

                print(
                    "##########################################"
                )


                # ---------------------------------------------
                # BRAKE
                # ---------------------------------------------

                trigger_brake_sequence()


                # Stop vision pipeline

                break


            # =================================================
            # DEBUG DISPLAY
            # =================================================

            debug_frame = frame.copy()


            # Draw road detection region

            height, width = (
                debug_frame.shape[:2]
            )


            road_y1 = int(
                ROAD_Y1_PERCENT *
                height
            )

            road_y2 = int(
                ROAD_Y2_PERCENT *
                height
            )


            cv2.rectangle(
                debug_frame,
                (0, road_y1),
                (width, road_y2),
                (255, 0, 0),
                2
            )


            # Draw detected road center

            if road_center is not None:

                cv2.circle(
                    debug_frame,
                    (
                        road_center,
                        int(
                            (
                                road_y1 +
                                road_y2
                            ) / 2
                        )
                    ),
                    8,
                    (0, 255, 0),
                    -1
                )


            # Display status

            if rash_flagged:

                status = (
                    "RASH DRIVING DETECTED"
                )

            else:

                status = (
                    "NORMAL DRIVING"
                )


            cv2.putText(
                debug_frame,
                status,
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )


            if speed_last_value is not None:

                cv2.putText(
                    debug_frame,
                    f"Speed: {speed_last_value:.1f} km/h",
                    (30, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2
                )


            cv2.imshow(
                "Slow Roads - MSS Capture",
                debug_frame
            )


            # Q = quit

            if (
                cv2.waitKey(1)
                &
                0xFF
            ) == ord("q"):

                break


    finally:

        # ALWAYS RELEASE SPACEBAR

        try:

            pydirectinput.keyUp(
                "space"
            )

        except:

            pass


        cap.release()

        cv2.destroyAllWindows()


        print()
        print(
            ">>> MSS CAMERA PIPELINE CLOSED <<<"
        )
        print()


# ============================================================
# MAIN
# ============================================================

def main():

    global running
    global alcohol_detected


    print()

    print(
        "=========================================="
    )

    print(
        " ALCOHOL + RASH DRIVING SYSTEM"
    )

    print(
        "=========================================="
    )

    print()

    print(
        "MSS MODE"
    )

    print(
        "IP CAMERA: DISABLED"
    )

    print()

    print(
        "Waiting for ALCOHOL DETECTED..."
    )

    print()


    try:

        while running:

            # -----------------------------------------------
            # ALCOHOL DETECTED
            # -----------------------------------------------

            if alcohol_detected:

                alcohol_detected = False


                print()
                print(
                    ">>> ALCOHOL DETECTED <<<"
                )

                print(
                    "Starting rash-driving detection..."
                )

                print()


                run_vision_pipeline()


                print(
                    "Returning to IDLE..."
                )

                print(
                    "Waiting for next ALCOHOL DETECTED..."
                )

                print()


            time.sleep(
                0.01
            )


    except KeyboardInterrupt:

        print()

        print(
            "Stopping system..."
        )


    finally:

        running = False


        try:

            pydirectinput.keyUp(
                "space"
            )

        except:

            pass


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
