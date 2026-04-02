# Isaac Sim + Nav2 — TurtleBot-like Example

A minimal, runnable example of a differential-drive robot navigating autonomously inside **NVIDIA Isaac Sim 4.x** using the **ROS2 Humble Nav2** navigation stack.

Both Isaac Sim and Nav2 run in Docker containers — no host install of either is required beyond the NVIDIA driver and Docker.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     HOST MACHINE (NVIDIA GPU + driver)                  │
│                                                                         │
│  ┌──────────────────────────┐        ┌────────────────────────────┐    │
│  │  isaac-sim container     │        │  ros2 container            │    │
│  │  nvcr.io/nvidia/         │        │  (built locally)           │    │
│  │    isaac-sim:5.1.0       │        │  ROS2 Humble + Nav2        │    │
│  │                          │        │                            │    │
│  │  Physics + LiDAR sim     │ ──────▶│  /scan /odom /tf           │    │
│  │  OmniGraph ROS2 bridge   │        │  AMCL + planner            │    │
│  │                          │◀────── │  /cmd_vel                  │    │
│  └──────────────────────────┘        └────────────────────────────┘    │
│           (needs GPU)                       (no GPU needed)             │
│                           host network                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| NVIDIA GPU | RTX or Quadro | Required for Isaac Sim physics |
| NVIDIA driver | ≥ 525 | Host driver only |
| [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) | latest | `nvidia-docker2` / `nvidia-ctk` |
| Docker + Compose | ≥ 24 | |
| NGC account | free | **NGC workflow only** — to pull `nvcr.io/nvidia/isaac-sim` |
| (optional) `xhost` | any | For RViz2 GUI |

### NGC login (NGC workflow only, one-time)

```bash
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key from https://ngc.nvidia.com/setup/api-key>
```

## Project Structure

```
isaac_sim_nav2/
├── isaac_sim/
│   ├── robot_config.py      # Robot physical parameters
│   └── setup_scene.py       # Isaac Sim Python script (run on host)
├── urdf/
│   └── turtlebot_like.urdf  # Robot URDF (used by robot_state_publisher)
├── config/
│   ├── nav2_params.yaml     # Full Nav2 parameter file
│   └── rviz2_nav2.rviz      # RViz2 config
├── maps/
│   ├── simple_map.pgm       # 200×200 occupancy grid (10×10 m arena)
│   ├── simple_map.yaml      # Map metadata
│   └── generate_map.py      # Script that generated the PGM
├── launch/
│   ├── nav2_bringup.launch.py   # Nav2 + AMCL (no RViz2)
│   └── full_system.launch.py    # Nav2 + RViz2
├── scripts/
│   ├── run_isaac_sim.sh     # Launch Isaac Sim simulation
│   └── send_goal.py         # Send a NavigateToPose goal to Nav2
├── docker/
│   ├── Dockerfile.ros2
│   └── entrypoint.sh
├── docker-compose.yml
└── package.xml
```

## Isaac Sim install options

Two workflows are supported. Pick one — all other steps are identical.

| | **NGC container** | **NVIDIA PyPI (pip)** |
|---|---|---|
| Source | `nvcr.io/nvidia/isaac-sim` | `pip install isaacsim` from `pypi.nvidia.com` |
| NGC account required | Yes | No |
| Download size | ~20 GB (one-time pull) | ~8 GB (one-time build) |
| Compose file | `docker-compose.yml` | `docker-compose.yml -f docker-compose.pypi.yml` |
| Isaac Sim launcher | `/isaac-sim/python.sh` | `python3` |

---

## Quick Start — NGC container (default)

### Step 1 — Build the ROS2 image and pull Isaac Sim (once)

```bash
cd isaac_sim_nav2

# Build the local ROS2/Nav2 image (~800 MB)
docker compose build ros2

# Pull Isaac Sim from NGC (~20 GB — get a coffee)
docker compose pull isaac-sim
```

### Step 2 — Start Isaac Sim headless (Terminal 1)

```bash
docker compose up isaac-sim
```

Isaac Sim boots, loads the scene, and starts publishing ROS2 topics.
Wait for the log line `[isaac_nav2] Simulation running`.

> **Want a visual?**
> The Isaac Sim container supports **WebRTC streaming** via [Omniverse Streaming Client](https://docs.omniverse.nvidia.com/streaming-client/latest/).
> Point it at `localhost:8211` once the container is running.

### Step 3 — Start Nav2 (Terminal 2)

```bash
docker compose run --rm ros2 \
  ros2 launch /workspace/launch/nav2_bringup.launch.py
```

Wait for: `[lifecycle_manager] All nodes are active`.

### Step 4 — Send a navigation goal (Terminal 3)

```bash
docker compose run --rm ros2 \
  python3 /workspace/scripts/send_goal.py --x 3.0 --y 2.0
```

The robot plans a path around the 5 obstacles and drives to (3.0, 2.0).

### Step 5 — Visualise with RViz2 (optional, needs a display)

```bash
xhost +local:docker
docker compose run --rm ros2 \
  ros2 launch /workspace/launch/full_system.launch.py
```

Use **2D Goal Pose** in RViz2 to click goals directly on the map.

---

## Quick Start — NVIDIA PyPI (no NGC account)

Uses `docker-compose.pypi.yml` as a Compose override that replaces the NGC
image with a locally built image that installs Isaac Sim via `pip` from
NVIDIA's PyPI index (`pypi.nvidia.com`).  No `docker login nvcr.io` required.

### Step 1 — Build both images (once)

```bash
cd isaac_sim_nav2

# Build the ROS2/Nav2 image (~800 MB)
docker compose build ros2

# Build the PyPI-based Isaac Sim image (~8 GB — get a coffee)
docker compose -f docker-compose.yml -f docker-compose.pypi.yml build isaac-sim
```

### Step 2 — Start Isaac Sim headless (Terminal 1)

```bash
docker compose -f docker-compose.yml -f docker-compose.pypi.yml up isaac-sim
```

### Step 3 — Start Nav2 (Terminal 2)

```bash
docker compose -f docker-compose.yml -f docker-compose.pypi.yml run --rm ros2 \
  ros2 launch /workspace/launch/nav2_bringup.launch.py
```

### Step 4 — Send a navigation goal (Terminal 3)

```bash
docker compose -f docker-compose.yml -f docker-compose.pypi.yml run --rm ros2 \
  python3 /workspace/scripts/send_goal.py --x 3.0 --y 2.0
```

### Step 5 — Visualise with RViz2 (optional, needs a display)

```bash
xhost +local:docker
docker compose -f docker-compose.yml -f docker-compose.pypi.yml run --rm ros2 \
  ros2 launch /workspace/launch/full_system.launch.py
```

> **Tip:** To avoid typing `-f docker-compose.yml -f docker-compose.pypi.yml` every time,
> set `COMPOSE_FILE` in your shell:
> ```bash
> export COMPOSE_FILE=docker-compose.yml:docker-compose.pypi.yml
> docker compose up isaac-sim   # now uses PyPI image automatically
> ```

---

## Host install alternative

`scripts/run_isaac_sim.sh` supports both install methods automatically.

**Option A — Omniverse Launcher** (sets `ISAAC_SIM_ROOT`, uses `python.sh`):

```bash
export ISAAC_SIM_ROOT=~/.local/share/ov/pkg/isaac-sim-5.1.0
bash scripts/run_isaac_sim.sh        # Terminal 1 (host)
docker compose run --rm ros2 \       # Terminal 2 (container)
  ros2 launch /workspace/launch/nav2_bringup.launch.py
```

**Option B — NVIDIA PyPI** (no `ISAAC_SIM_ROOT` needed, uses `python3`):

```bash
pip install "isaacsim[all,extscache]==5.1.0" "isaacsim-replicator==5.1.0.0" \
    --extra-index-url https://pypi.nvidia.com
bash scripts/run_isaac_sim.sh        # Terminal 1 (host, auto-detects pip install)
docker compose run --rm ros2 \       # Terminal 2 (container)
  ros2 launch /workspace/launch/nav2_bringup.launch.py
```

Both approaches use `ROS_DOMAIN_ID=42` over the host network — no extra bridging needed.

---

## How It Works

### Isaac Sim side (`isaac_sim/setup_scene.py`)

1. **World setup**: flat 10×10 m ground plane, 5 box obstacles
2. **Robot**: procedural USD articulation — chassis + 2 driven wheels (revolute joints + `DriveAPI`) + 2 passive casters
3. **Sensor**: RTX 2-D LiDAR (360°, 3.5 m range, 10 Hz) mounted at `base_link/lidar`
4. **OmniGraph bridge** publishes:
   - `/scan` — `sensor_msgs/LaserScan`
   - `/odom` — `nav_msgs/Odometry`
   - `/tf`   — odom → base_footprint transform
5. **OmniGraph bridge** subscribes:
   - `/cmd_vel` → `DifferentialController` → wheel drive joints

### ROS2 / Nav2 side

| Node | Role |
|---|---|
| `robot_state_publisher` | Publishes URDF → `/tf` (static transforms) |
| `map_server` | Publishes `/map` from `simple_map.pgm` |
| `amcl` | Localises robot on the map using `/scan` + `/odom` |
| `planner_server` (NavFn) | Computes global path to goal |
| `controller_server` (DWB) | Follows path, avoids obstacles, sends `/cmd_vel` |
| `behavior_server` | Recovery behaviours (spin, back-up) |
| `bt_navigator` | Orchestrates the BT navigate_to_pose action |

### Map

`simple_map.pgm` is a 200×200 px PGM image (0.05 m/px → 10×10 m).
Obstacles in the map are aligned with the Isaac Sim box positions.
Regenerate with `python3 maps/generate_map.py` if you change the obstacles.

---

## Customisation

### Change robot speed
Edit `config/nav2_params.yaml`:
```yaml
FollowPath:
  max_vel_x: 0.26   # m/s forward
  max_vel_theta: 1.0  # rad/s rotation
```
And `isaac_sim/robot_config.py`:
```python
"max_linear_vel":  0.26,
"max_angular_vel": 1.0,
```

### Add obstacles
1. Append to `OBSTACLE_SPECS` in `isaac_sim/setup_scene.py`
2. Re-add the same obstacle to `maps/generate_map.py` `OBSTACLES` list
3. Regenerate the map: `python3 maps/generate_map.py`

### Run headless (no GUI)
```bash
bash scripts/run_isaac_sim.sh --headless
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `pull access denied` for isaac-sim | NGC workflow: run `docker login nvcr.io` with your NGC API key |
| `could not select device driver nvidia` | Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) |
| `python.sh not found` (host install) | Set `ISAAC_SIM_ROOT` to your Omniverse install dir, or switch to the PyPI install |
| `No matching distribution found for isaacsim` | Add `--extra-index-url https://pypi.nvidia.com` to your pip command |
| PyPI image build fails with CUDA errors | Ensure the host NVIDIA driver is ≥ 525 and the Container Toolkit is installed |
| `/scan` not received in Nav2 | Check `ROS_DOMAIN_ID=42` is set in both containers (it is by default) |
| AMCL not converging | Click **2D Pose Estimate** in RViz2 to give it the starting position |
| Robot spins in place | Increase `update_min_d` in `amcl` params |
| `Goal REJECTED` | Nav2 lifecycle may not be fully active yet; wait ~5 seconds after launch |
| Isaac Sim OOM (GPU) | Reduce RTX ray count or use a smaller GPU memory profile in `setup_scene.py` |
