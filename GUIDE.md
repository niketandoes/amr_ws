# AMR Warehouse Simulation Guide

This workspace contains a multi-robot simulation using ROS 2, Gazebo, and Nav2. This guide explains how to start the simulation and what each script and launch file does.

## 🚀 Quick Start: How to Run the Simulation

The entire simulation (Gazebo, Nav2 for all robots, Coordinators, and the Web Dashboard) is now bundled into a single launch file.

**Terminal 1 (Main Simulation):**
```bash
# 1. Navigate to the workspace and build (if you haven't already)
cd ~/amr_ws
colcon build

# 2. Source the ROS 2 workspace
source install/setup.bash

# 3. Launch the simulation
ros2 launch amr_simulation multi_robot_warehouse.launch.py
```
*Wait ~10-15 seconds for Gazebo to render and the Nav2 nodes to become fully active.*

**View the Dashboard:**
Once the simulation is running, open your web browser and go to: **[http://localhost:8080](http://localhost:8080)**

**Terminal 2 (Run a Test Mission):**
To make the robots actually do something, you run a test script in a new terminal.
```bash
cd ~/amr_ws
source install/setup.bash

# Run the coordinated choke point test (robots navigate safely)
ros2 run amr_simulation coordinated_choke_test.py
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
| **`multi_robot_warehouse.launch.py`** | **The Main Entrypoint.** This launches *everything*: Gazebo, the AMRs, Nav2 for each robot, the twist_mux velocity arbiters, the decentralized coordinators, the intent broadcasters, battery simulators, the task allocator, and the fleet dashboard web server. |
| **`nav2_bringup.launch.py`** | Brings up the Nav2 navigation stack (map server, controller, planner, behavior server) for a *single* AMR namespace. This is called dynamically by the `multi_robot_warehouse.launch.py` script. |
| **`warehouse_world.launch.py`** | A simpler launch file that only starts the Gazebo world. (Usually, you don't need to run this directly since the multi-robot launch includes it). |

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
