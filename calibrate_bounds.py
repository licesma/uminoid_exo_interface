import sys
import os
import time
import tty
import termios
import select

import yaml
import numpy as np
from InquirerPy import inquirer

from vr import VuerTeleop

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "third_party", "inspire_hands"))
from inspire_hand import InspireHand
from inspire_hand.hand import FingerID, Register

CONFIG_FILE = "./assets/inspire_hand/inspire_hand.yml"
BOUNDS_FILE = "hand_bounds.yaml"
HISTORY_LEN = 6


def load_bounds():
    with open(BOUNDS_FILE) as f:
        return yaml.safe_load(f)


def save_bounds(bounds):
    with open(BOUNDS_FILE, "w") as f:
        yaml.dump(bounds, f, default_flow_style=False)


def get_finger_value(left_hand, right_hand, hand_side, finger_name):
    hand = left_hand if hand_side == "left" else right_hand
    if hand is None:
        return 0.0
    return getattr(hand, finger_name).wrist_to_tip


def read_key_nonblocking():
    """Return the next keypress if one is waiting, else None."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def calibrate_loop(teleop, hand_side, finger_name):
    """
    Live calibration loop.  Returns True when the user goes back ('b').
    In raw-terminal mode so single keypresses are caught without Enter.
    """
    history = []

    while True:
        head, left_wrist, right_wrist, left_hand, right_hand = teleop.step()
        value = get_finger_value(left_hand, right_hand, hand_side, finger_name)

        history.append(value)
        if len(history) > HISTORY_LEN:
            history.pop(0)

        bounds = load_bounds()
        low  = bounds[hand_side][finger_name]["low"]
        high = bounds[hand_side][finger_name]["high"]

        key = read_key_nonblocking()
        if key == 'b':
            return
        elif key == 'l':
            bounds[hand_side][finger_name]["low"] = float(value)
            save_bounds(bounds)
            low = float(value)
        elif key == 'u':
            bounds[hand_side][finger_name]["high"] = float(value)
            save_bounds(bounds)
            high = float(value)

        # Refresh display
        os.system("clear")
        print(f"Hand: {hand_side}   Finger: {finger_name}\n")
        print("'b' -> go back   'l' -> set lower bound   'u' -> set upper bound")
        print(f"lower bound: {low:.4f},   upper bound: {high:4f}\n")

        for v in history:
            print(f"{v:.4f}")

        time.sleep(0.05)


def main():
    hand = InspireHand(port="/dev/ttyUSB0", baudrate=115200, slave_id=1)
    hand.open()
    teleop = VuerTeleop(config_file_path=CONFIG_FILE, img_shm_name=None)

    try:
        while True:
            os.system("clear")
            hand_side = inquirer.select(
                message="Select hand:",
                choices=["left", "right"],
            ).execute()

            while True:
                os.system("clear")
                print(f"Hand: {hand_side}\n")

                finger_name = inquirer.select(
                    message="Select finger:",
                    choices=["index", "middle", "ring", "pinky", "← back"],
                ).execute()

                if finger_name == "← back":
                    break

                # Switch stdin to raw mode only during the calibration loop
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    calibrate_loop(teleop, hand_side, finger_name)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        hand.close()
        teleop.shutdown()


if __name__ == "__main__":
    main()
