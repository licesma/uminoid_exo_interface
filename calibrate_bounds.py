import sys
import os
import time
import tty
import termios
import select

import serial
import yaml
from InquirerPy import inquirer

from joints import ARM_JOINTS, JOINT_NAMES
from utils.convert_to_radian import convert_to_radian
from constants import ENCODER_PRECISION_RAD

SERIAL_PORT  = "/dev/pts/7"
BAUD_RATE    = 115200

ROBOT_BOUNDS_FILE = "./cpp/g1/model/upperBodyJointBounds.yaml"
JOINT_BOUNDS_FILE = "./cpp/joint_reader/upperBodyReaderBounds.yaml"


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)


def joint_display_name(name):
    return " ".join(word.capitalize() for word in name.split("_"))


def joint_status_badge(side, joint, robot_bounds, joint_bounds):
    full_name = f"{side}_{joint}"
    robot_lower = robot_bounds[full_name]["lower"]
    robot_upper = robot_bounds[full_name]["upper"]
    jl = joint_bounds[full_name]["lower"]
    ju = joint_bounds[full_name]["upper"]
    delta = abs((robot_upper - robot_lower) - (ju - jl))
    if delta < 2 * ENCODER_PRECISION_RAD:
        return "[correct]"
    elif delta < 4 * ENCODER_PRECISION_RAD:
        return "[close]"
    else:
        return "[incorrect]"


def read_key_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def read_serial_value(ser, joint_index):
    """Flush stale buffered data, then return the freshest encoder value for joint_index."""
    try:
        ser.reset_input_buffer()
        line = ser.readline().decode("ascii").strip()
        parts = line.split(",")
        if len(parts) == 15:           # t_us + 14 encoder values
            return float(parts[joint_index + 1])
    except Exception:
        pass
    return None


def calibrate_loop(ser, side, joint):
    full_name   = f"{side}_{joint}"
    joint_index = JOINT_NAMES.index(full_name)

    robot_bounds = load_yaml(ROBOT_BOUNDS_FILE)
    robot_lower  = robot_bounds[full_name]["lower"]
    robot_upper  = robot_bounds[full_name]["upper"]

    joint_bounds = load_yaml(JOINT_BOUNDS_FILE)

    current_value = None

    while True:
        v = read_serial_value(ser, joint_index)
        if v is not None:
            current_value = v

        key = read_key_nonblocking()
        if key == "b":
            return
        elif key == "l" and current_value is not None:
            joint_bounds[full_name]["lower"] = convert_to_radian(current_value)
            save_yaml(JOINT_BOUNDS_FILE, joint_bounds)
        elif key == "u" and current_value is not None:
            joint_bounds[full_name]["upper"] = convert_to_radian(current_value)
            save_yaml(JOINT_BOUNDS_FILE, joint_bounds)

        jl = joint_bounds[full_name]["lower"]
        ju = joint_bounds[full_name]["upper"]

        robot_range = robot_upper - robot_lower
        joint_range = ju - jl
        delta_range = abs(robot_range - joint_range)

        if delta_range < 2 * ENCODER_PRECISION_RAD:
            color = "\033[32m"   # green
        elif delta_range < 4 * ENCODER_PRECISION_RAD:
            color = "\033[33m"   # yellow
        else:
            color = "\033[31m"   # red
        reset = "\033[0m"
        delta_tag = f"{color}[\u0394 Range {delta_range:.4f}]{reset}"

        os.system("clear")
        print(f"Side: {side.capitalize()},  Joint: {joint_display_name(joint)}  {delta_tag}\n")
        print(
            f"{'Robot Limits:':<16}  "
            f"Lower: {robot_lower:<22.4f}  "
            f"Upper: {robot_upper:<22.4f}  "
            f"Range: {robot_range:.4f}"
        )
        print(
            f"{'Joint Limits:':<16}  "
            f"Lower: {jl:<22.4f}  "
            f"Upper: {ju:<22.4f}  "
            f"Range: {joint_range:.4f}"
        )
        print()
        if current_value is not None:
            cv_rad = convert_to_radian(current_value)
            cv_str = f"{cv_rad:.4f} rad  (raw: {current_value:.0f})"
        else:
            cv_str = "—"
        print(f"Current value: {cv_str}")
        print()
        print("'b' -> go back   'l' -> set lower bound   'u' -> set upper bound")

        time.sleep(0.05)


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

    try:
        while True:
            os.system("clear")
            side = inquirer.select(
                message="Select side:",
                choices=["left", "right"],
            ).execute()

            while True:
                os.system("clear")
                print(f"Side: {side.capitalize()}\n")

                robot_bounds = load_yaml(ROBOT_BOUNDS_FILE)
                joint_bounds = load_yaml(JOINT_BOUNDS_FILE)
                joint_choices = [
                    {
                        "name": f"{j}  {joint_status_badge(side, j, robot_bounds, joint_bounds)}",
                        "value": j,
                    }
                    for j in ARM_JOINTS
                ] + [{"name": "← back", "value": "← back"}]

                joint = inquirer.select(
                    message="Select joint:",
                    choices=joint_choices,
                ).execute()

                if joint == "← back":
                    break

                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    calibrate_loop(ser, side, joint)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
