#!/usr/bin/env python3
"""CrazySim smoke test: prove the SITL drone is a drop-in stand-in for the real
Crazyflie.

It does exactly the plumbing the lab does -- scan, connect over cflib, set the
PID controller, wait for the onboard position estimator, then take off / hover /
land -- so if this works, the lab notebook can talk to the simulator the same
way it would talk to the drone.

This is NOT a Lab 2 solution: it flies a fixed hover at a fixed height. No RRT,
no measured obstacle course, nothing from the assignment.

Usage
-----
Terminal 1 (starts the firmware + MuJoCo viewer):
    pixi run -e sim sim

Terminal 2:
    pixi run -e sim python worlds/mujoco/hello_crazyflie.py

Pass a different cflib URI as the first argument for a swarm instance, e.g.
``udp://127.0.0.1:19851``.
"""
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

URI = sys.argv[1] if len(sys.argv) > 1 else "udp://127.0.0.1:19850"
HEIGHT = 0.5          # hover height, m
RATE = 0.1            # setpoint period, s (same 10 Hz loop the lab uses)


def scan():
    print("Scanning interfaces for Crazyflies...")
    found = [entry[0] for entry in cflib.crtp.scan_interfaces()]
    if found:
        print("Crazyflies found:")
        for uri in found:
            print(f"    {uri}")
    else:
        print("No Crazyflies found -- is the simulator running (`pixi run -e sim sim`)?")
    return found


def wait_for_position_estimator(scf, timeout=30.0):
    """Block until the Kalman variances settle (mirrors the lab's Part IV)."""
    print("Waiting for estimator to find position...")
    log_config = LogConfig(name="Kalman Variance", period_in_ms=500)
    for axis in "XYZ":
        log_config.add_variable(f"kalman.varP{axis}", "float")

    history = {axis: [1000.0] * 10 for axis in "XYZ"}
    threshold = 0.001
    deadline = time.time() + timeout
    with SyncLogger(scf, log_config) as logger:
        for _, data, _ in logger:
            for axis in "XYZ":
                history[axis].append(data[f"kalman.varP{axis}"])
                history[axis].pop(0)
            spread = {axis: max(h) - min(h) for axis, h in history.items()}
            print("    ", {axis: round(v, 5) for axis, v in spread.items()})
            if all(v < threshold for v in spread.values()):
                print("Estimator converged.")
                return True
            if time.time() > deadline:
                print("Timed out waiting for the estimator to converge.")
                return False
    return False


def send_for(send_fn, seconds, *args):
    """Call a commander setpoint at RATE for `seconds` (setpoints must stream)."""
    for _ in range(int(seconds / RATE)):
        send_fn(*args)
        time.sleep(RATE)


def start_z_monitor(cf):
    """Log stateEstimate.z asynchronously (non-blocking); returns the value list."""
    zs = []
    log_config = LogConfig(name="z", period_in_ms=50)
    log_config.add_variable("stateEstimate.z", "float")
    log_config.data_received_cb.add_callback(
        lambda _ts, data, _lc: zs.append(data["stateEstimate.z"])
    )
    cf.log.add_config(log_config)
    log_config.start()
    return log_config, zs


def main():
    cflib.crtp.init_drivers(enable_debug_driver=False)
    scan()

    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        z_log, zs = start_z_monitor(cf)

        # Same controller/estimator setup as the lab's Part IV.
        cf.param.set_value("stabilizer.controller", "1")
        cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        cf.param.set_value("kalman.resetEstimation", "0")
        wait_for_position_estimator(scf)

        print(f"Ascending to {HEIGHT} m ...")
        for i in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, (i + 1) / 20 * HEIGHT)
            time.sleep(RATE)

        print("Hovering ...")
        send_for(cf.commander.send_hover_setpoint, 3.0, 0, 0, 0, HEIGHT)

        print("Descending ...")
        for i in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, HEIGHT * (10 - i) / 10)
            time.sleep(RATE)

        # Keep streaming stop setpoints after touchdown. The commander watchdog
        # (2 s) fires the moment the stream stops, and while the drone is still
        # inside its "flying" window (thrust over idle within the last 2 s) the
        # supervisor latches into "Locked, reboot required" -- which needs a
        # firmware restart. Streaming until isFlying clears avoids that.
        print("Landing (streaming stop setpoints) ...")
        send_for(cf.commander.send_stop_setpoint, 6.0)
        z_log.stop()

    print(f"Reached {max(zs) if zs else float('nan'):.3f} m (target {HEIGHT} m)")
    print("Done!")


if __name__ == "__main__":
    main()
