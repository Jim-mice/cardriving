import socket
import time
from pynput import keyboard

CAR = ("192.168.137.204", 7788)
SEND_HZ = 20.0
STEER_HOLD_THRESHOLD_MS = 300
STEER_TAP_MS = 350
SPEED = (0, 80, 100, 150)

def resolve(gear, steer):
    base = SPEED[abs(gear)]
    if gear == 0:
        return 0, 0
    signed = base if gear > 0 else -base
    delta = base * 20 // 100
    if steer < 0:
        return signed - delta, signed + delta
    if steer > 0:
        return signed + delta, signed - delta
    return signed, signed

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gear = 0
    admin = 0
    left_down = right_down = False
    left_t = right_t = 0.0
    seq = ge = le = re = ae = 0
    steer = 0
    next_tx = 0.0
    print("HOST REMOTE CONTROL\nUP/DOWN: gear  LEFT/RIGHT: steer  SPACE: admin stop  ESC: quit")

    def send():
        nonlocal seq
        seq += 1
        msg = (f"RC1 seq={seq} gear={gear} steer={steer} admin={admin} "
               f"ge={ge} le={le} re={re} ae={ae}")
        sock.sendto(msg.encode("ascii"), CAR)

    running = True
    def update_steer(now):
        nonlocal steer
        if left_down and right_down:
            steer = 0
        elif left_down:
            steer = -1
        elif right_down:
            steer = 1
        elif left_t and (now - left_t) * 1000 < STEER_TAP_MS:
            steer = -1
        elif right_t and (now - right_t) * 1000 < STEER_TAP_MS:
            steer = 1
        else:
            steer = 0

    def on_press(key):
        nonlocal gear, admin, left_down, right_down, left_t, right_t, ge, le, re, ae, running
        now = time.monotonic()
        if key == keyboard.Key.esc:
            gear = 0; admin = 1; left_down = right_down = False; running = False
        elif key == keyboard.Key.up and not getattr(on_press, "up", False):
            on_press.up = True; gear = min(gear + 1, 3); ge += 1
        elif key == keyboard.Key.down and not getattr(on_press, "down", False):
            on_press.down = True; gear = max(gear - 1, -3); ge += 1
        elif key == keyboard.Key.left and not left_down:
            left_down = True; left_t = now; le += 1
        elif key == keyboard.Key.right and not right_down:
            right_down = True; right_t = now; re += 1
        elif key == keyboard.Key.space and not getattr(on_press, "space", False):
            on_press.space = True; admin = 0 if admin else 1; ae += 1
            if admin: left_down = right_down = False

    def on_release(key):
        nonlocal left_down, right_down
        if key == keyboard.Key.up: on_press.up = False
        elif key == keyboard.Key.down: on_press.down = False
        elif key == keyboard.Key.space: on_press.space = False
        elif key == keyboard.Key.left: left_down = False
        elif key == keyboard.Key.right: right_down = False

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    try:
        while running:
            now = time.monotonic()
            update_steer(now)
            if next_tx <= now:
                send(); next_tx = now + 1.0 / SEND_HZ
            time.sleep(0.005)
    finally:
        gear = 0; steer = 0; admin = 1
        for _ in range(3): send(); time.sleep(0.03)
        listener.stop()
        sock.close()

if __name__ == "__main__":
    main()
