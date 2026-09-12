# AMR Warehouse Simulation Guide

This workspace contains a multi-robot simulation using ROS 2, Gazebo, and Nav2. This guide explains how to start the simulation and what each script and launch file does.

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

### Option B: Modular Launch (Recommended for Debugging)
Open separate terminals for each component so logs don't get mixed up.

**Terminal 1 (Gazebo):**
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

**Terminal 2 (Run a Test Mission):**
To make the robots actually do something, you run a test script in a new terminal.

```bash
cd ~/amr_ws
source install/setup.bash

# Option A: If you launched MULTIPLE robots (e.g. Option A above)
ros2 run amr_simulation coordinated_choke_test.py

# Option B: If you launched a SINGLE robot (e.g. Option B above)
ros2 run amr_simulation navigate_choke_point.py
```

> [!TIP]
> **Cleanup:** If Gazebo or ROS nodes hang, you can forcefully clean them up before a fresh launch using:
> `pkill -9 -f "gazebo|gzserver|gzclient|ros_gz_bridge|nav2|bt_navigator|component_container"`

---

## 📁 File Reference

There are many Python files in this workspace. Here is a breakdown of what each one is for.

### Launch Files (`src/amr_simulation/launch/`)

These files orchestrate starting multiple ROS 2 nodes at once.

| File | Purpose |
|---|---|
| **`multi_robot_warehouse.launch.py`** | **The Main Entrypoint.** Includes all the modular launch files below to launch everything at once. You can toggle specific robots off using arguments (e.g., `launch_amr2:=false`). |
| **`gazebo_environment.launch.py`** | Launches only the Gazebo simulation world (`warehouse.sdf`) and the global clock bridge. Run this first when debugging! |
| **`single_amr.launch.py`** | Brings up everything for a *single* robot namespace, including Nav2, the state publisher, Gazebo spawner, intent broadcaster, and coordinator. Pass `namespace`, `initial_pose_x`, `initial_pose_y`, and `initial_pose_yaw`. |
| **`fleet_dashboard.launch.py`** | A simple launch file that starts only the web dashboard server. |
| **`nav2_bringup.launch.py`** | Brings up the Nav2 navigation stack (map server, controller, planner, behavior server) for a single namespace. Called dynamically by `single_amr.launch.py`. |
| **`warehouse_world.launch.py`** | A simpler launch file that only starts the Gazebo world. |

### Scripts (`src/amr_simulation/scripts/`)

These are the individual ROS 2 nodes that handle logic, coordination, and testing.

#### Core Fleet Management
| File | Purpose |
|---|---|
| **`decentralized_coordinator.py`** | **Peer-to-Peer Arbitration.** Runs on each robot. Uses a zone reservation protocol to negotiate right-of-way through the narrow warehouse choke points. Yields cleanly by cancelling Nav2 goals and resuming them when the corridor clears. |
| **`intent_broadcaster.py`** | **Heartbeat Generator.** Broadcasts the robot's current pose and planned future Nav2 waypoints to all other robots so the coordinators can evaluate conflicts. |
| **`task_allocator_cnp.py`** | **Contract Net Protocol (CNP).** Dynamically distributes tasks to the AMRs based on a bidding system (e.g., closest robot with enough battery wins the task). |
| **`battery_simulator.py`** | Simulates battery consumption while driving and charging behavior when idle at a charging station. |
| **`fleet_dashboard_server.py`** | **Web Backend.** Subscribes to fleet telemetry and serves a live HTML/JS dashboard at `http://localhost:8080`. |

#### Tests and Missions
| File | Purpose |
|---|---|
| **`coordinated_choke_test.py`** | **Primary Test.** Dispatches goals to force AMRs to cross the warehouse choke point simultaneously, demonstrating the decentralized coordinator safely managing the traffic. |
| **`uncoordinated_choke_test.py`** | **Failure Test.** Dispatches goals but simulates a scenario where coordination fails, demonstrating a deadlock/crash when robots try to enter the corridor at the same time. |
| **`navigate_choke_point.py`** | Simple action client to send a single robot across the map. |
| **`benchmark_runner.py` / `benchmark_logger.py`** | Utilities for running repeatable performance benchmarks and logging metrics like completion time and delays. |

#### Utilities
| File | Purpose |
|---|---|
| **`generate_map.py`** | Utility script used offline to convert the 3D Gazebo `.sdf` world into a 2D occupancy grid map (`.yaml`/`.pgm`) used by Nav2. |
