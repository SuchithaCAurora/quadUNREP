# uav_trajectory_generator (`trajectory_generator_ros2`)

This repository is modified from ACL: https://github.com/kotakondo/uav_trajectory_generator 

This branch has tracking law and CBF for relative station keeping.

For the UNREP Demo:
build all packages as described below
Choose which leader trajectory in the /config/default.yaml file
Choose follower behaviors in the /config/follower_behavior.yaml file
Choose any px4 configurations in the /config/px4_config_template.yaml file

Run: python3 uav_trajectory_generator/src/ros2_px4_stack/scripts/tmux/sitl_tmux_multi.py
## Figure-8 SITL demo

https://github.com/user-attachments/assets/8b980866-85d8-44ee-91a5-5b93790eceba

PX4 SITL + Gazebo Classic (iris) flying the figure-8 produced by this
package. Follow the instructions below to reproduce it locally. A raw
download of the same clip is also attached to the
[v0.1-demo release](https://github.com/kotakondo/uav_trajectory_generator/releases/tag/v0.1-demo).

## Overview

ROS 2 port of ACL's trajectory generator. The node publishes smooth
`snapstack_msgs2/Goal` trajectories (Circle, Figure8, Square, Rectangle, Line,
Boomerang, Reciprocating, Bounce, T, I, M) that a MAVROS-based offboard
bridge forwards to PX4 as setpoints.

This README documents how to stand up the full stack — PX4 SITL +
Gazebo-Classic + MAVROS + this package — from a blank Ubuntu 22.04 / ROS 2
Humble box, and fly a Figure-8 in simulation.

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** — install per <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>
- **Gazebo Classic 11** (not `gz sim`):
  ```bash
  sudo apt install gazebo libgazebo-dev
  ```

### apt packages

```bash
sudo apt install \
  ros-humble-mavros ros-humble-mavros-msgs ros-humble-mavros-extras \
  ros-humble-mavlink \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  python3-vcstool
```

(`python3-vcstool` provides `vcs`, used below to pin the sibling package
versions.)

Install the MAVROS GeographicLib geoid dataset (MAVROS crashes on launch
without it):

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### Python packages

```bash
pip install pymavlink six numpy scipy
```

## Clone the workspace

```bash
mkdir -p ~/code/uav_trajectory_simulator_ws/src && cd ~/code/uav_trajectory_simulator_ws/src
git clone git@github.com:kotakondo/uav_trajectory_generator.git
vcs import < uav_trajectory_generator/uav_trajectory_generator.repos
```

`vcs import` reads the pinned SHAs from
`uav_trajectory_generator/uav_trajectory_generator.repos` and clones the
four sibling packages at those exact commits. Everything lands flat under
`src/`:

- `uav_trajectory_generator` — the C++ trajectory generator (this package).
- `snapstack_msgs2` — shared message defs (`Goal`, `State`, `QuadFlightMode`).
- `mission_mode` — `MissionModeChange` service.
- `behavior_selector2` — rqt GUI with GO / END / KILL buttons.
- `ros2_px4_stack` — Python bridge: Goal → MAVROS offboard setpoints, plus
  the arm + OFFBOARD kick.

**Bumping a dep later**: `cd src/<pkg>`, pull, copy the new HEAD SHA into
`uav_trajectory_generator.repos`, and commit the `.repos` change in
`uav_trajectory_generator`. Downstream users re-run `vcs import` to sync.

## Clone PX4-Autopilot (outside the workspace)

PX4 has its own `make`-based build and must **not** live under `src/`.

```bash
cd ~/code
git clone --recursive -b dev git@github.com:kotakondo/PX4-Autopilot.git
```

The `dev` branch in the fork carries:

- `ROMFS/.../10015_gazebo-classic_iris` — failsafe relaxations
  (`COM_DISARM_PRFLT`, `COM_DISARM_LAND`, `COM_OF_LOSS_T`) required for
  offboard-trajectory SITL to not disarm mid-flight.
- A submodule pointer to `kotakondo/PX4-SITL_gazebo-classic` (also `dev`)
  with a plain-grey ground plane and the aruco Gazebo plugin disabled
  (the plugin otherwise needs `opencv_contrib`).

## Build

Conda can silently hijack the C++ compiler (`~/anaconda3/bin/cl`) and the
CMake Boost path — deactivate fully before building:

```bash
while [ "${CONDA_SHLVL:-0}" -gt 0 ]; do conda deactivate; done
source /opt/ros/humble/setup.bash
```

### Build PX4 SITL (first run: ~3 min)

```bash
cd ~/code/PX4-Autopilot
make px4_sitl gazebo-classic_iris     # launches once, Ctrl-C after you see "Ready for takeoff!"
```

Running it once forces the first build. You do not need to keep this session
open; the tmux helper below will launch SITL on its own.

### Build the ROS 2 workspace

```bash
cd ~/code/uav_trajectory_simulator_ws
colcon build --symlink-install
source install/setup.bash
```

## Run a Figure-8 in SITL

Set the trajectory type in
`src/uav_trajectory_generator/config/default.yaml`:

```yaml
traj_type: Figure8    # Circle | Figure8 | Square | Rectangle | Line | ...
```

Figure8 reuses the Circle block (`r`, `center_x`, `center_y`, `v_goals`,
`t_traj`, `circle_accel`). No rebuild is needed — the YAML is re-read at
node startup.

Launch the whole stack in one tmux session:

```bash
python3 src/ros2_px4_stack/scripts/tmux/sitl_tmux.py
```

Six panes will come up in order:

| Pane | Waits | What |
|---|---|---|
| 0 | 0 s  | PX4 SITL + Gazebo-Classic (iris) |
| 1 | 12 s | MAVROS bridged to PX4's UDP 14580 |
| 2 | 15 s | `trajectory_generator_ros2` — publishes `/SQ01/goal` |
| 3 | 17 s | `ros2_px4_stack`'s `track_gen_traj_py` — arm + OFFBOARD kick + MAVROS setpoint relay |
| 4 | 17 s | `behavior_selector2` rqt GUI (GO / END / KILL) |
| 5 | 20 s | `ros2 topic echo /SQ01/goal` |

Once the rqt window appears, click **GO**. Pane 3 should log:

```
Waiting for trajectory setpoints...
set PX4 param COM_DISARM_PRFLT = -1.0
Setpoints flowing; priming OFFBOARD (1s)...
armed state changed from False to True
mode changed from AUTO.LOITER to OFFBOARD
PX4 mode=OFFBOARD armed=True
```

The iris lifts off, climbs to `alt: 1.8` m, and flies the figure-8.
**END** lands, **KILL** cuts motors.

Detach from tmux with `Ctrl-b d`; tear the session down with
`tmux kill-session -t SQ01_sitl`.

### Switching trajectory

Edit `config/default.yaml` → change `traj_type` → restart the session. Each
shape's parameters live in a labeled block in the same YAML (Circle uses
`r` / `v_goals`, Square uses `side_length` / `square_accel`, etc.).

## Troubleshooting

**"unknown target 'gz_x500'"** — your PX4 build has only Gazebo-Classic
targets, not Gazebo Harmonic. Use `gazebo-classic_iris` (the default in the
tmux helper).

**Build fails in sitl_gazebo-classic with missing `GSTREAMER_APP_LIBRARIES`**
— install `libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev`.

**Build fails with `fatal error: opencv2/aruco.hpp: No such file`** — you
have OpenCV without `opencv_contrib`. The `dev` branch of the forked
submodule already disables this plugin; make sure PX4-Autopilot was cloned
with `--recursive -b dev`.

**MAVROS crashes with `GeographicLib exception: File not readable
egm96-5.pgm`** — run
`sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh` once.

**CMake errors mentioning `~/anaconda3/...`** — conda is still on `PATH`
even after `conda deactivate`. Fully deactivate:
`while [ "${CONDA_SHLVL:-0}" -gt 0 ]; do conda deactivate; done`.

**Drone arms then disarms after ~10 s ("auto preflight disarming")** — PX4
thinks it never took off. This is already handled by the `dev`-branch
airframe override; if you are on upstream PX4, cherry-pick the
`10015_gazebo-classic_iris` commit or set `COM_DISARM_PRFLT=-1` manually.

## Architecture (one-paragraph version)

`behavior_selector2` GUI → `change_mode` service → `behavior_selector_node`
publishes `QuadFlightMode` on `/globalflightmode`. `trajectory_generator_ros2`
runs a `GROUND → TAKING_OFF → HOVERING → INIT_POS_TRAJ → TRAJ_FOLLOWING →
LANDING` state machine and publishes `snapstack_msgs2/Goal` on `<ns>/goal`
at `pub_freq` Hz. `ros2_px4_stack`'s `track_gen_traj_py` repacks each Goal
into a `trajectory_msgs/MultiDOFJointTrajectory` and publishes it on
`mavros/setpoint_trajectory/local` at 100 Hz, while separately arming the
vehicle and switching PX4 to OFFBOARD once setpoints are flowing.
