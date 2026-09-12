# AMR Warehouse Simulation Guide

This workspace contains a multi-robot simulation using ROS 2, Gazebo, and Nav2. This guide explains how to start the simulation, what each script and launch file does, custom interfaces, configuration parameters, and benchmark reporting.

## 🚀 Quick Start: How to Run the Simulation

The simulation components have been modularized so you can run them individually for easier debugging and isolated logs, or all at once.

### Option A: Launch Everything Together
**Terminal 1:**
```bash
cd ~/amr_ws
colcon build
source install/setup.bash
ros2 launch amr_simulation multi_robot_warehouse.launch.py
```

> [!TIP]
> **Modifying Interfaces:** If you modify any custom ROS 2 messages (`.msg`) in the `amr_interfaces` package (e.g., `TaskAward.msg` or `FleetIntent.msg`), you MUST run `colcon build --symlink-install --packages-select amr_interfaces amr_simulation` to regenerate the Python/C++ bindings, followed by `source install/setup.bash` in all active terminals, otherwise the nodes will fail to communicate.

### Option B: Modular Launch (Recommended for Debugging)
Open separate terminals for each component so logs don't get mixed up.

**Terminal 1 (Gazebo & Core World):**
```bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_simulation gazebo_environment.launch.py
```

**Terminal 2 (Robot 1):**
```bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_simulation single_amr.launch.py namespace:=amr1 initial_pose_x:=-4.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

**Terminal 3 (Dashboard):**
```bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_simulation fleet_dashboard.launch.py
```

*Wait ~10-15 seconds for Gazebo to render and the Nav2 nodes to become fully active.*

**View the Dashboard:**
Once running, open your web browser and go to: **[http://localhost:8080](http://localhost:8080)**

**Terminal 4 (Run a Test Mission):**
To make the robots actually do something, you run a test script in a new terminal.

```bash
cd ~/amr_ws
source install/setup.bash

# Option A: If you launched MULTIPLE robots (e.g. Option A above)
ros2 run amr_simulation coordinated_choke_test.py

# Option B: If you launched a SINGLE robot (e.g. Option B above)
ros2 run amr_simulation navigate_choke_point.py
```

### Option C: RViz2 Visualization Commands
To visualize robot models (`amr1`, `amr2`, `amr3`), TF transforms (`/tf`), costmaps, laser scans, and active Nav2 trajectories:

**1. Launch Everything with RViz2 in One Command:**
```bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_simulation multi_robot_warehouse.launch.py launch_rviz:=true
```

**2. Open RViz2 in a New Terminal while Simulation is Already Running:**
```bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_simulation rviz.launch.py
```
*(Loads `multi_amr.rviz` preconfigured with robot models and colored laser scans for `amr1`, `amr2`, and `amr3`)*

**3. Launch RViz2 for Single Robot or Custom Config:**
```bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_simulation rviz.launch.py rviz_config:=src/amr_simulation/config/view_amr.rviz
```

**4. Launch Single Robot World bringing up RViz2 automatically:**
```bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_simulation warehouse_world.launch.py rviz:=true
```

> [!NOTE]
> **Benchmark Logging:** Whenever multi-robot launches or `warehouse_world.launch.py` are executed, the `benchmark_logger` node automatically records performance metrics (task completion time, travel distance, minimum robot clearance, and deadlock duration) and exports them to `~/amr_ws/benchmark_results.csv`.

> [!TIP]
> **Automatic Process Cleanup:** The simulation launch files (`multi_robot_warehouse.launch.py` and `warehouse_world.launch.py`) now automatically detect and kill lingering Nav2 zombie processes (`/opt/ros/humble/lib/nav2_*`) before spawning new instances to prevent duplicate node conflicts. For a complete manual teardown of Gazebo and ROS bridges, you can run:
> `pkill -9 -f "gazebo|gzserver|gzclient|ros_gz_bridge|nav2|bt_navigator|component_container"`

---

## 📁 File Reference

### Launch Files (`src/amr_simulation/launch/`)

These files orchestrate starting multiple ROS 2 nodes at once.

| File | Launch Command / Arguments | Purpose |
|---|---|---|
| **`multi_robot_warehouse.launch.py`** | `ros2 launch amr_simulation multi_robot_warehouse.launch.py` | **The Main Entrypoint.** Includes all modular launch files to bring up the world, robots (`amr1`, `amr2`, `amr3`), web dashboard, and the benchmark logger. Toggles: `launch_amr1:=true`, `launch_amr2:=true`, `launch_amr3:=true`, `launch_dashboard:=true`, `launch_rviz:=true`. |
| **`rviz.launch.py`** | `ros2 launch amr_simulation rviz.launch.py [rviz_config:=...] [namespace:=...]` | **RViz2 Visualization.** Brings up RViz2 initialized with `view_amr.rviz` configuration for monitoring fleet TF trees, costmaps, laser scans, and global path planners. |
| **`warehouse_world.launch.py`** | `ros2 launch amr_simulation warehouse_world.launch.py [rviz:=true]` | Brings up the Gazebo world, robot state publisher, entity spawner, ROS-GZ bridges, Nav2 stack, `twist_mux`, `benchmark_logger`, and optionally launches RViz2 when `rviz:=true` is passed. |
| **`gazebo_environment.launch.py`** | `ros2 launch amr_simulation gazebo_environment.launch.py` | Launches only the Gazebo simulation world (`warehouse.sdf`) and global clock bridge. Run this first when debugging! |
| **`single_amr.launch.py`** | `ros2 launch amr_simulation single_amr.launch.py namespace:=amr1 initial_pose_x:=-4.0 ...` | Brings up everything for a *single* robot namespace (`{name}/{key}` namespaced Nav2 params, state publisher, Gazebo spawner, intent broadcaster, coordinator). |
| **`fleet_dashboard.launch.py`** | `ros2 launch amr_simulation fleet_dashboard.launch.py` | Starts the web dashboard backend node (`fleet_dashboard_server.py`). |
| **`nav2_bringup.launch.py`** | *Called internally by single_amr / warehouse_world* | Brings up the Nav2 navigation stack for a single namespace. Remaps `cmd_vel` output to `cmd_vel_nav` for velocity multiplexing through `twist_mux`. |

### Scripts (`src/amr_simulation/scripts/`)

These are the individual ROS 2 nodes that handle logic, coordination, metrics, and testing.

#### Core Fleet Management
| File | Purpose |
|---|---|
| **`decentralized_coordinator.py`** | **Peer-to-Peer Corridor Arbitration.** Runs on each robot. Evaluates narrow choke point collisions (corridor bounds: X `[-0.8, 0.8]`, Y `[-0.60, 0.60]`, approach Y `1.2`). Uses fair request-timestamp tie-breaking, tracks entry verification (`peer_has_entered_zone`), and yields right-of-way cleanly by issuing override velocity commands (`cmd_vel_coord`). |
| **`intent_broadcaster.py`** | **Heartbeat Generator.** Broadcasts current pose, planned Nav2 waypoints, and operational state (`0: IDLE`, `1: APPROACHING`, `2: IN_CHOKE`, `3: YIELDING`). Listens to `cmd_vel_coord` to accurately report yielding halts. |
| **`task_allocator_cnp.py`** | **Contract Net Protocol (CNP).** Bidding and auction manager. Broadcasts auctions, evaluates bids based on distance/battery, and awards tasks with dynamic target poses (`geometry_msgs/Pose target_pose`). Includes auction retries (up to 2 attempts before standby fallback) and suppresses false blockage detection while yielding near choke points. |
| **`battery_simulator.py`** | Simulates battery drain during navigation and charging state while idle at charging stations. |
| **`fleet_dashboard_server.py`** | **Web Backend.** Subscribes to fleet telemetry and serves the live web dashboard at `http://localhost:8080`. Resolves asset/map paths using share directory and relative source fallbacks. |

#### Tests, Metrics, and Missions
| File | Purpose |
|---|---|
| **`benchmark_logger.py`** | **Automated Metrics Recorder.** Monitors goal status transitions via UUIDs, tracks TF2 odometry travel distance and velocity, detects deadlocks, and automatically exports results to `~/amr_ws/benchmark_results.csv`. |
| **`benchmark_runner.py`** | Automated test harness for executing batch benchmark scenarios across multiple runs. |
| **`coordinated_choke_test.py`** | **Primary Coordination Test.** Sends conflicting navigation tasks forcing AMRs to negotiate and cross the narrow corridor safely using decentralized coordination. |
| **`uncoordinated_choke_test.py`** | **Failure Baseline Test.** Dispatches goals without collision negotiation to demonstrate traffic deadlocks when uncoordinated. |
| **`navigate_choke_point.py`** | Simple action client for dispatching a single robot goal across the warehouse. |

#### Utilities
| File | Purpose |
|---|---|
| **`generate_map.py`** | Utility script to generate 2D occupancy grid maps (`.yaml`/`.pgm`) from 3D Gazebo `.sdf` worlds. |

---

## 🛠️ Architecture, Interfaces & Configuration

### Custom Message Interfaces (`src/amr_interfaces/msg/`)
- **`TaskAward.msg`**: Extended to include `geometry_msgs/Pose target_pose` along with `task_id` and `winner_id`, allowing winning bidders to receive dynamic destination targets directly from the auctioneer.
- **`FleetIntent.msg`**: Broadcasts `robot_id`, `current_pose`, `current_velocity`, `planned_waypoints`, `is_in_choke_zone`, `request_timestamp`, and `current_state`.
- **`TaskAuction.msg`** & **`TaskBid.msg`**: Support Contract Net Protocol bidding rounds between AMRs.

### Velocity Multiplexing (`twist_mux`)
Nav2 node outputs velocity commands to `cmd_vel_nav`. The `twist_mux` node multiplexes control inputs according to priority (`config/twist_mux_config.yaml`) and publishes to `cmd_vel`, allowing the coordinator (`cmd_vel_coord`) to safely override or pause movement when yielding right-of-way.

### Nav2 Costmap & Pathfinding Tuning
- **Inflation Radius**: Set to `0.22` in `config/nav2_params.yaml` (for both local and global costmaps) to allow robots to navigate through narrow 1.2m warehouse corridors without excessive inflation cost blockage.
- **Collision Detection**: `use_collision_detection` disabled in controller server (`RPurePursuit`) with `max_allowed_time_to_collision_up_to_carrot: 0.5` to prevent false local controller halts inside tight choke points where the peer-to-peer coordinator handles traffic safety.
