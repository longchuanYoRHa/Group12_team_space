#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
One-shot calibration script for MyCobot280 on Raspberry Pi.

Goal:
- After a reboot, run this script ONCE.
- It will:
  1) print connection info and basic status
  2) clear errors/queue
  3) try to RELAX joints reliably (power_off fallback)
  4) let you manually move the arm to the desired pose
  5) re-enable servos and HOLD the pose
  6) read and print get_angles/get_coords multiple times
  7) optionally save results to a timestamped JSON file

Notes:
- If something else is holding the serial port (/dev/ttyAMA0), this script may behave oddly.
- power_off may make the arm drop due to gravity. HOLD the arm before confirming.
"""

import json
import os
import sys
import time
from datetime import datetime

from pymycobot import MyCobot280
from pymycobot import PI_PORT, PI_BAUD


def now_str():
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def is_angles(x):
    return isinstance(x, list) and len(x) == 6 and all(isinstance(v, (int, float)) for v in x)


def is_coords(x):
    return isinstance(x, list) and len(x) == 6 and all(isinstance(v, (int, float)) for v in x)


def safe_call(name, fn, *args, **kwargs):
    try:
        v = fn(*args, **kwargs)
        print(f"{name} -> {v}")
        return v
    except Exception as e:
        print(f"{name} !! exception: {type(e).__name__}: {e}")
        return None


def safe_get_angles(mc, tag):
    v = None
    try:
        v = mc.get_angles()
    except Exception as e:
        print(f"{tag} get_angles exception: {type(e).__name__}: {e}")
        return None
    if not is_angles(v):
        print(f"{tag} get_angles invalid: {v}")
        return None
    return v


def safe_get_coords(mc, tag):
    v = None
    try:
        v = mc.get_coords()
    except Exception as e:
        print(f"{tag} get_coords exception: {type(e).__name__}: {e}")
        return None
    if not is_coords(v):
        print(f"{tag} get_coords invalid: {v}")
        return None
    return v


def print_readings(mc, rounds=3, interval=0.2):
    readings = []
    for i in range(rounds):
        angles = safe_get_angles(mc, f"[read {i+1}]")
        coords = safe_get_coords(mc, f"[read {i+1}]")
        print(f"[read {i+1}] angles = {angles}")
        print(f"[read {i+1}] coords  = {coords}")
        readings.append({"angles": angles, "coords": coords})
        time.sleep(interval)
    return readings


def try_relax(mc):
    """
    Try to relax joints in a robust order.
    Returns a dict describing what happened.
    """
    info = {"relax_method": None, "is_power_on_before": None, "is_power_on_after": None}

    info["is_power_on_before"] = safe_call("is_power_on(before)", mc.is_power_on)

    # Step 1: clear errors/queue
    safe_call("clear_error_information", mc.clear_error_information)
    safe_call("clear_queue", mc.clear_queue)
    time.sleep(0.2)

    # Step 2: set free mode (may not actually relax torque on your firmware)
    safe_call("set_free_mode(1)", mc.set_free_mode, 1)
    time.sleep(0.2)

    # Step 3: try release_all_servos (may or may not work depending on firmware)
    safe_call("release_all_servos", mc.release_all_servos)
    time.sleep(0.3)

    # Check if it actually disabled servos
    servo_en = safe_call("is_all_servo_enable(after release)", mc.is_all_servo_enable)
    if servo_en == 0:
        info["relax_method"] = "release_all_servos"
        info["is_power_on_after"] = safe_call("is_power_on(after release)", mc.is_power_on)
        return info

    # Step 4 (fallback): power_off should relax torque most reliably
    safe_call("power_off", mc.power_off)
    time.sleep(0.6)
    info["relax_method"] = "power_off"
    info["is_power_on_after"] = safe_call("is_power_on(after power_off)", mc.is_power_on)
    return info


def restore_and_hold(mc):
    """
    Re-enable servos and hold current pose.
    """
    safe_call("power_on", mc.power_on)
    time.sleep(0.5)
    safe_call("focus_all_servos", mc.focus_all_servos)
    time.sleep(0.6)
    safe_call("set_free_mode(0)", mc.set_free_mode, 0)
    time.sleep(0.2)

    safe_call("is_power_on(final)", mc.is_power_on)
    safe_call("is_all_servo_enable(final)", mc.is_all_servo_enable)
    safe_call("get_error_information(final)", mc.get_error_information)


def main():
    print("=== MyCobot280 one-shot calibration ===")
    print("Time:", now_str())
    print("PI_PORT =", PI_PORT)
    print("PI_BAUD =", PI_BAUD)
    print()

    # Create connection
    mc = MyCobot280(PI_PORT, PI_BAUD)

    # Print initial status
    safe_call("get_error_information(init)", mc.get_error_information)
    safe_call("is_power_on(init)", mc.is_power_on)
    safe_call("is_all_servo_enable(init)", mc.is_all_servo_enable)
    safe_call("is_free_mode(init)", mc.is_free_mode)
    print()

    # Relax phase
    print("=== Step 1/3: Relax joints ===")
    print("Safety: HOLD the arm before it relaxes (it may drop).")
    input("Press Enter to start relaxing...")
    relax_info = try_relax(mc)
    print("relax_info =", relax_info)
    print()

    print("Now the arm SHOULD be relaxed. If it is NOT relaxed, do NOT force it.")
    print("If it is relaxed, manually move the arm to your desired calibration pose.")
    input("After you finish manual positioning, press Enter to lock & read pose...")
    print()

    # Restore/hold phase
    print("=== Step 2/3: Restore servos and hold pose ===")
    restore_and_hold(mc)
    print()

    # Read pose multiple times
    print("=== Step 3/3: Read angles/coords (multiple samples) ===")
    readings = print_readings(mc, rounds=5, interval=0.2)

    # Save results
    out = {
        "timestamp": now_str(),
        "pi_port": PI_PORT,
        "pi_baud": PI_BAUD,
        "relax_info": relax_info,
        "readings": readings,
    }
    out_dir = os.path.join(os.getcwd(), "calib_outputs")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"mycobot280_calib_{now_str()}.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    print()
    print("Saved:", out_path)
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)