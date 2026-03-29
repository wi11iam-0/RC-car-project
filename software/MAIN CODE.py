import threading
import time
import subprocess
import sys
import os
import re
from dataclasses import dataclass

import pygame
import numpy as np
import cv2
from websocket import WebSocketApp, ABNF

#  CONFIG 
ESP32_WS_URL = "ws://192.168.137.24:81"   # WS port 81
#192.168.43.226 for phone hotspot
stop_event = threading.Event()

#  CAMERA 
frame_queue = __import__("queue").Queue(maxsize=1)


def cv_display_thread_fn(window_name="ESP32-CAM"):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    frame_count = 0
    start_time = time.time()
    fps = 0.0

    while not stop_event.is_set():
        try:
            jpeg_bytes = frame_queue.get(timeout=0.5)
        except __import__("queue").Empty:
            # keep GUI responsive
            cv2.waitKey(1)
            continue

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        # FPS calc
        frame_count += 1
        now = time.time()
        elapsed = now - start_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            start_time = now

        cv2.putText(
            frame, f"FPS = {int(round(fps))}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA
        )

        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) == 27:  # closes camera
            stop_event.set()
            break

    cv2.destroyAllWindows()


#  CONTROLLER STATE

@dataclass
class ControllerState:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    cross: int = 0
    circle: int = 0
    square: int = 0
    triangle: int = 0
    dpad_x: int = 0
    dpad_y: int = 0


controller_state_lock = threading.Lock()
controller_state: ControllerState | None = None


# regex for controller via pygame. output line
controller_line_re = re.compile(
    r"LX=(?P<LX>[+-]?\d+\.\d+)\s+"
    r"LY=(?P<LY>[+-]?\d+\.\d+)\s+"
    r"RX=(?P<RX>[+-]?\d+\.\d+)\s+"
    r"RY=(?P<RY>[+-]?\d+\.\d+)\s+"
    r"Throttle=(?P<Throttle>\d+\.\d+)\s+"
    r"Brake=(?P<Brake>\d+\.\d+)\s+"
    r"X=(?P<X>[01])\s+"
    r"O=(?P<O>[01])\s+"
    r"\[]=(?P<Square>[01])\s+"
    r"/\\=(?P<Triangle>[01])\s+"
    r"DPAD=\((?P<DX>-?1|0|1),(?P<DY>-?1|0|1)\)"
)


def controller_reader_thread_fn():
    """
    Robust reader: reads ONE character at a time from controller script stdout,
    splitting records on '\r' or '\n' (controller prints end='\r').
    """
    global controller_state

    script_path = os.path.join(os.path.dirname(__file__), "controller via pygame.py")
    print(f"[CTRL] Starting controller reader. script_path={script_path}")

    if not os.path.isfile(script_path):
        print(f"[CTRL] ERROR: script not found: {script_path}")
        stop_event.set()
        return

    try:
        proc = subprocess.Popen(
            [sys.executable, script_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            text=True,
        )
        print(f"[CTRL] Controller script started. pid={proc.pid}")
    except Exception as e:
        print(f"[CTRL] Failed to start controller script: {e}")
        stop_event.set()
        return

    line = ""

    try:
        while not stop_event.is_set():
            ch = proc.stdout.read(1)
            if not ch:
                if proc.poll() is not None:
                    print(f"[CTRL] Controller script exited. code={proc.returncode}")
                    stop_event.set()
                    break
                time.sleep(0.01)
                continue

            if ch == "\r" or ch == "\n":
                line = line.strip()
                if line:
                    m = controller_line_re.search(line)
                    if m:
                        try:
                            cs = ControllerState(
                                lx=float(m.group("LX")),
                                ly=float(m.group("LY")),
                                rx=float(m.group("RX")),
                                ry=float(m.group("RY")),
                                throttle=float(m.group("Throttle")),
                                brake=float(m.group("Brake")),
                                cross=int(m.group("X")),
                                circle=int(m.group("O")),
                                square=int(m.group("Square")),
                                triangle=int(m.group("Triangle")),
                                dpad_x=int(m.group("DX")),
                                dpad_y=int(m.group("DY")),
                            )
                            with controller_state_lock:
                                controller_state = cs
                        except ValueError:
                            pass
                line = ""
            else:
                line += ch

    finally:
        try:
            proc.terminate()
        except Exception:
            pass
        print("[CTRL] Controller reader exiting")


#  WEBSOCKET HANDLERS

def on_open(ws):
    print("[WS] Connected")


def on_close(ws, code, reason):
    print(f"[WS] Closed: code={code}, reason={reason}")
    stop_event.set()


def on_error(ws, error):
    print(f"[WS] Error: {error}")


def on_message(ws, message):
    # optional: print ESP32 text messages
    if isinstance(message, str) and message.strip():
        print(f"[WS <- ESP32] {message}")


# CAMERA
def on_data(ws, data, data_type, cont):
    """
    Receives BINARY frames from ESP32 (JPEG).
    Extracts clean JPEG between SOI/EOI and keeps newest only.
    """
    if data_type != ABNF.OPCODE_BINARY:
        return

    raw = data
    start = raw.find(b"\xff\xd8")
    end = raw.rfind(b"\xff\xd9")
    if start == -1 or end == -1 or end <= start:
        return

    jpeg = raw[start:end + 2]

    try:
        if not frame_queue.empty():
            frame_queue.get_nowait()  # drop old
        frame_queue.put_nowait(jpeg)
    except Exception:
        pass


def ws_thread_fn():
    # pings help some networks
    wsapp.run_forever(ping_interval=10, ping_timeout=5)
    stop_event.set()


#  START THREADS

wsapp = WebSocketApp(
    ESP32_WS_URL,
    on_open=on_open,
    on_message=on_message,
    on_data=on_data,  
    on_close=on_close,
    on_error=on_error,
    header=["Sec-WebSocket-Extensions:"],  # disable compression
)

t_ws = threading.Thread(target=ws_thread_fn, daemon=True)
t_ws.start()

t_ctrl = threading.Thread(target=controller_reader_thread_fn, daemon=True)
t_ctrl.start()

t_cv = threading.Thread(target=cv_display_thread_fn, daemon=True)  
t_cv.start()


#  PYGAME (MOUSE) + SEND LOOP
pygame.init()
width, height, fps = 500, 500, 60
timer = pygame.time.Clock()
screen = pygame.display.set_mode([width, height])
pygame.display.set_caption("Mouse + Controller Sender (WS)")

SEND_INTERVAL = 0.25  # seconds
last_send_time = 0.0

run = True

try:
    while run and not stop_event.is_set():
        timer.tick(fps)
        screen.fill("white")

        mx, my = pygame.mouse.get_pos()
        pygame.draw.circle(screen, "red", (mx, my), 10)

        now = time.time()

        if now - last_send_time >= SEND_INTERVAL:
            last_send_time = now

            with controller_state_lock:
                cs = controller_state

            if cs is None:
                print("[MAIN] controller_state is None (no parsed controller data yet)")
            else:
                print(
                    f"[CTRL->SEND] "
                    f"LX={cs.lx:+.2f} LY={cs.ly:+.2f}  "
                    f"RX={cs.rx:+.2f} RY={cs.ry:+.2f}  "
                    f"Throttle={cs.throttle:.2f} Brake={cs.brake:.2f}  "
                    f"X={cs.cross} O={cs.circle} []={cs.square} /\\={cs.triangle}  "
                    f"DPAD=({cs.dpad_x},{cs.dpad_y})"
                )

            if wsapp and wsapp.sock and wsapp.sock.connected:
                mouse_string = f"M:{mx},{height - my}"
                try:
                    wsapp.send(mouse_string)
                except Exception as e:
                    print(f"[WS] Mouse send error: {e}")

                if cs is not None:
                    controller_string = (
                        "C:"
                        f"{cs.lx:.3f},"
                        f"{cs.ly:.3f},"
                        f"{cs.rx:.3f},"
                        f"{cs.ry:.3f},"
                        f"{cs.throttle:.3f},"
                        f"{cs.brake:.3f},"
                        f"{cs.cross},"
                        f"{cs.circle},"
                        f"{cs.square},"
                        f"{cs.triangle},"
                        f"{cs.dpad_x},"
                        f"{cs.dpad_y}"
                    )
                    print(f"[WS -> ESP32] {controller_string}")
                    try:
                        wsapp.send(controller_string)
                    except Exception as e:
                        print(f"[WS] send error: {e}")

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        pygame.display.flip()
        time.sleep(0.005)

except KeyboardInterrupt:
    stop_event.set()

finally:
    stop_event.set()
    try:
        wsapp.close()
    except Exception:
        pass
    pygame.quit()
    cv2.destroyAllWindows()
    print("[MAIN] Exiting")
