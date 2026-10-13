# worlds/mujoco — simulated Crazyflie (CrazySim SITL)

A drop-in stand-in for the physical Crazyflie used in the ROB 345/549 labs, so
the lab notebooks can talk to a drone without hardware.

[CrazySim](https://github.com/gtfactslab/CrazySim) is a **software-in-the-loop**
simulator: the *real* Crazyflie firmware is compiled to run on this machine, and
[cflib](https://github.com/bitcraze/crazyflie-lib-python) talks to it over UDP
exactly as it would talk to a radio. MuJoCo provides the physics + viewer; no
Gazebo and no ROS are involved.

Everything here is driven by the `sim` environment defined in the repo-root
[`pixi.toml`](../../pixi.toml).

## Layout

```
worlds/mujoco/
├── README.md                 # this file
├── hello_crazyflie.py        # smoke test: scan → connect → take off → land
├── scene_course.xml          # example Lab 2 pipe course (MuJoCo scene)
├── lab_course.py             # the same course as Part III numbers + sanity check
└── dependencies/
    └── CrazySim/             # external clone (git-ignored)
        └── crazyflie-firmware/   # SITL firmware + MuJoCo launch scripts
```

## One-time setup

```bash
# from the repo root
git clone https://github.com/gtfactslab/CrazySim.git worlds/mujoco/dependencies/CrazySim
cd worlds/mujoco/dependencies/CrazySim
git submodule update --init crazyflie-firmware
cd crazyflie-firmware
git submodule update --init --recursive     # vendor/*, tools/crazyflie-simulation, drone-models
cd -                                         # back to the repo root

pixi install -e sim        # py3.11 + mujoco + cflib
pixi run -e sim sim-build  # compiles the SITL firmware (cf2)
```

## Run

Two terminals, both from the repo root:

```bash
# Terminal 1 — firmware + MuJoCo viewer
pixi run -e sim sim

# Terminal 2 — cflib smoke test
pixi run -e sim python worlds/mujoco/hello_crazyflie.py
```

The drone listens for cflib on **`udp://127.0.0.1:19850`**. `scan_interfaces()`
auto-discovers it (cflib's UDP driver probes ports 19850–19859). A swarm uses
19851, 19852, … — pass the URI to the smoke test, e.g.
`... hello_crazyflie.py udp://127.0.0.1:19851`.

To point the lab notebook at the simulator, use that `udp://…` URI where it
builds its `radio://0/<n>/2M` URI.

## Example Lab 2 course

`scene_course.xml` is a 3 m × 3 m "netted cage" (orange tape) with seven PVC
pipes, a start marker at the origin and a goal square at (2.55, 2.55). It is the
sim twin of the Part III numbers in `lab_course.py`, so the notebook plans in
exactly the world the simulator flies in.

```bash
# Terminal 1 — firmware + viewer, with the course loaded
pixi run -e sim sim-course
#   (equivalent: pixi run -e sim sim -- -s "$PWD/worlds/mujoco/scene_course.xml")

# Terminal 2 — print the Part III block + a connectivity check
pixi run -e sim python worlds/mujoco/lab_course.py
```

Frame: the drone spawns at world (0, 0) facing +X and zeroes its estimator
there, so the lab's arena frame (origin = start, +x forward, +y left) **is** the
MuJoCo world frame — no coordinate conversion needed for this course. Pipes are
0.05 m radius in the scene and appear inflated to 0.20 m in the planner numbers.

Useful launch-script flags (pass to the task, e.g.
`pixi run -e sim sim -- --sensor-noise`):
`--sensor-noise`, `--ground-effect`, `--flowdeck`, `--wind-speed`, `--turbulence`,
`-s scene_obstacles.xml`, `-m cf2x_L250|cf2x_T350|cf21B_500`.

## Notes / gotchas

- **Keep streaming setpoints until the drone has landed.** The commander has a
  2 s watchdog; if the stream stops while the drone is still inside its "flying"
  window (thrust above idle within the last 2 s), the supervisor latches into
  `Locked, reboot required` — a persistent fault that needs a firmware restart,
  and the *next* run won't take off (arming does not help). So don't cut the
  stream right after touchdown: keep sending `send_stop_setpoint` until
  `supervisor.info` bit 4 (`isFlying`) clears. `hello_crazyflie.py` streams stop
  setpoints for 6 s after the descent, which is enough to fly repeatedly. If it
  ever does lock, restart the sim (the launch script kills a stale `cf2`).
- **Clear a stale simulator before restarting.** The upstream launch script only
  `pkill`s `cf2`, not `crazysim.py`. If the sim was killed uncleanly, a leftover
  `crazysim.py` keeps port 19950 and the next `pixi run -e sim sim` dies with
  `Address already in use`. Run `pkill -f '[c]razysim.py'` first.
- **The sim env is Python 3.11**, separate from the py3.9 lab env, because
  `cflib` requires Python ≥ 3.10.
- **Compiler compat.** The SITL firmware predates GCC 14, which turned several
  formerly-warning C idioms into errors. `pixi.toml` sets `CFLAGS` in the `sim`
  environment to downgrade them back to warnings. That only takes effect on a
  **fresh** `sitl_make/build/`, so if a build ever goes sideways:
  `rm -rf worlds/mujoco/dependencies/CrazySim/crazyflie-firmware/sitl_make/build`
  and re-run `pixi run -e sim sim-build`.
- **`make cf2`, not `make all`.** `make all` also builds the Gazebo plugin
  (`crazysim_gz`), which needs Gazebo and isn't used by the MuJoCo backend.
- The MuJoCo viewer needs a display. Set `--vis` off (edit the launch script) for
  a headless run.
- `dependencies/` is git-ignored; `MUJOCO_LOG.TXT` is too.
