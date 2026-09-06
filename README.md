# Edge-AI AMR Fleet Coordination: Decentralized Multi-Agent System

A fully decentralized multi-robot autonomous navigation and fleet coordination system developed on **ROS 2** and **Nav2**. All coordination occurs peer-to-peer (P2P) across the Data Distribution Service (DDS) mesh layer without centralized fleet servers or message brokers.

---

## Implemented Architecture & System Overview

The system features three differential-drive Autonomous Mobile Robots (`amr1`, `amr2`, `amr3`) operating in an enclosed 12m &times; 12m warehouse environment featuring a central 1.15m narrow bottleneck.

```
+-------------------------------------------------------------------------+
|                       FLAT P2P DDS MULTICAST MESH                       |
|                                                                         |
|  +---------------------+   +---------------------+   +---------------+  |
|  |     /fleet/intent   |   | /fleet/task_auction |   | /fleet/task   |  |
|  | (FleetIntent.msg)   |   | (TaskAuction.msg)   |   |   _bids/award |  |
|  +----------+----------+   +----------+----------+   +-------+-------+  |
+-------------|-------------------------|----------------------|----------+
              |                         |                      |
     +--------+--------+       +--------+--------+    +--------+--------+
     |      AMR 1      |       |      AMR 2      |    |      AMR 3      |
     | - Nav2 Stack    |       | - Nav2 Stack    |    | - Nav2 Stack    |
     | - Broadcaster   |       | - Broadcaster   |    | - Broadcaster   |
     | - Coordinator   |       | - Coordinator   |    | - Coordinator   |
     | - CNP Allocator |       | - CNP Allocator |    | - CNP Allocator |
     | - Battery Sim   |       | - Battery Sim   |    | - Battery Sim   |
     +-----------------+       +-----------------+    +-----------------+
              |                         |                      |
              +-------------------------+----------------------+
                                        |
                         +--------------v--------------+
                         |    FLEET DASHBOARD SERVER   |
                         |  (HTTP & SSE Telemetry 8080)|
                         +--------------+--------------+
                                        |
                         +--------------v--------------+
                         |  LIVE WEB DASHBOARD (HTML5) |
                         |  2D Canvas / Telemetry / CNP|
                         +-----------------------------+
```

---

## Completed Implementations

### 1. Multi-Robot Simulation World & Isolated Nav2 Stacks
- **Gazebo Sim Warehouse (`warehouse.sdf`)**: Enclosed 12m &times; 12m arena with static warehouse racks, boundary walls, and a central choke point wall with a 1.15m passage at $(0, 0)$. Surface friction parameters configured to eliminate odometric drift.
- **Parametric Robot Description (`amr.xacro`)**: Differential-drive robot model parameterized with `robot_name` for namespace isolation. Includes 360&deg; 10 Hz LiDAR ray sensor plugins and differential drive odometry publishers.
- **Isolated Nav2 Stacks (`nav2_bringup.launch.py`)**: Dedicated lifecycle navigation stacks launched per robot under isolated namespaces (`/amr1`, `/amr2`, `/amr3`) with independent global/local costmaps, planners, Regulated Pure Pursuit controllers, and recovery behaviors.

### 2. Peer-to-Peer Intent Sharing (Mesh Network)
- **Interface (`amr_interfaces/msg/FleetIntent.msg`)**: Encapsulates robot ID, odometry pose, linear/angular velocities, downsampled planned waypoints ($t+1\text{s}, t+2\text{s}, t+3\text{s}$), and current state flags.
- **Broadcaster Node (`intent_broadcaster.py`)**: Runs locally on each robot, publishing a 5 Hz heartbeat to `/fleet/intent` over DDS using `BEST_EFFORT` reliability and `VOLATILE` durability QoS.

### 3. Decentralized Conflict Arbitration & Choke-Point Reservation
- **Coordinator Node (`decentralized_coordinator.py`)**: Evaluates real-time spatial proximities to the 1.15m bottleneck at 50 Hz.
- **Right-of-Way Arbitration**: When opposing AMRs enter the choke detection zone, deterministic tie-breaking grants passage to the lower string ID (`amr1` over `amr2`), while the yielding AMR holds position outside the corridor ($|x| \ge 3.5\text{ m}$) until the bottleneck is cleared.

### 4. Blocked-Aisle Task Allocation via Contract Net Protocol (CNP)
- **Interfaces (`TaskAuction.msg`, `TaskBid.msg`, `TaskAward.msg`)**: Custom message definitions for decentralized auctioning, bidding, and awarding.
- **Allocator Node (`task_allocator_cnp.py`)**:
  - **Blockage Detection**: Automatically detects stalled navigation ($v < 0.05\text{ m/s}$ for $>3.0\text{ s}$ during an active goal) or accepts synthetic fault injection.
  - **Decentralized Auction**: Stalled AMR flags `TASK_BLOCKED` and broadcasts a `TaskAuction` on `/fleet/task_auction`.
  - **Heuristic Bidding**: Peer robots compute a composite cost score:
    $$\text{Cost} = (1.0 \times \text{Distance}) + (0.5 \times (100.0 - \text{Battery\_SOC}))$$
    and submit bids on `/fleet/task_bids`.
  - **Task Award & Reroute**: The lowest bidder is awarded the task and dispatches a Nav2 goal, while the blocked AMR pulls over into a standby holding bay to clear the lane.

### 5. Velocity-Scaled Synthetic Battery Simulation
- **Simulator Node (`battery_simulator.py`)**: Publishes standard `sensor_msgs/msg/BatteryState` at 1.0 Hz on `/{robot_name}/battery_state`.
- **Dynamic Depletion**: Discharges proportionally based on odometric velocity (linear and angular speed) with a baseline idle draw when stationary. Automatically maps percentage to cell voltages (21.0 V to 25.2 V for 6S LiPo).

### 6. Live Fleet Web Dashboard & Telemetry Bridge
- **Dashboard Server (`fleet_dashboard_server.py`)**: Zero-dependency Python HTTP and Server-Sent Events (SSE) server running on `http://localhost:8080`. Aggregates odometry, battery states, choke status, and CNP events at 10 Hz.
- **Frontend (`dashboard/`)**:
  - **2D Canvas Map (`app.js`)**: Interactive top-down view of the warehouse, racks, 1.15m choke zone, live AMR coordinates, orientation arrows, and motion breadcrumbs.
  - **Telemetry Cards (`index.html`, `style.css`)**: Live display of $(x, y)$ positions, linear speed, heading angle, choke distance, and animated battery SOC bars.
  - **CNP Live Event Feed**: Real-time event log displaying auction announcements, competing bids with calculated heuristic costs, and winner awards.
  - **Mission Control**: Buttons to dispatch head-to-head opposing tests, simulate AMR 1 blockages, and reset fleet breadcrumbs.

### 7. Automated Benchmarking & Comparative Evaluation
- **Benchmark Suite (`benchmark_runner.py`, `benchmark_logger.py`)**: Records trial outcomes, total fleet completion time, deadlock durations, minimum inter-robot clearances, and auction turnarounds directly to `benchmark_results.csv`.
- **Verified Results**:
  - **Condition A (Uncoordinated Baseline)**: 88.9% deadlock/abort rate with simultaneous opposing entries.
  - **Condition B (Decentralized Coordination + CNP)**: 100% traversal success rate, zero collisions ($d < 0.5\text{ m}$), sub-500ms auction turnaround (mean 485.2 ms), and ~32.4% fleet throughput efficiency improvement.

---

## Workspace Structure

```
amr_ws/
├── .gitignore                          # Ignores build/, install/, log/, and temporary files
├── README.md                           # System documentation
├── Detailed.md                         # Technical protocol and specification
├── benchmark_results.csv               # Historical benchmark trial logs
└── src/
    ├── amr_interfaces/                 # Custom ROS 2 Interface Definitions
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── FleetIntent.msg         # P2P heartbeat (pose, velocity, plan waypoints, state)
    │       ├── TaskAuction.msg         # CNP task auction announcement
    │       ├── TaskBid.msg             # CNP candidate peer cost bid
    │       └── TaskAward.msg           # CNP task assignment announcement
    │
    └── amr_simulation/                 # Main Simulation, Coordination & Dashboard Package
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   ├── nav2_params.yaml        # Nav2 parameters (costmaps, controller, planner)
        │   └── navigate_to_pose.xml    # Nav2 behavior tree configuration
        ├── dashboard/                  # Live Fleet Web Dashboard Frontend
        │   ├── index.html              # HTML5 responsive UI structure
        │   ├── style.css               # Dark-mode glassmorphic design system
        │   └── app.js                  # 2D Canvas renderer & SSE telemetry client
        ├── launch/
        │   ├── warehouse_world.launch.py       # Single-robot baseline launch
        │   ├── nav2_bringup.launch.py          # Namespaced Nav2 stack launch
        │   └── multi_robot_warehouse.launch.py # Full multi-AMR simulation master launch
        ├── maps/
        │   ├── warehouse_map.yaml      # Map metadata and resolution
        │   └── warehouse_map.pgm       # 2D occupancy grid image
        ├── models/
        │   └── amr.xacro               # Parameterized differential-drive robot URDF/Xacro
        ├── scripts/
        │   ├── battery_simulator.py         # Dynamic battery depletion node
        │   ├── benchmark_logger.py          # Odometry & deadlock CSV logger
        │   ├── benchmark_runner.py          # Automated multi-trial benchmark evaluator
        │   ├── coordinated_choke_test.py    # Head-to-head opposing dispatch client
        │   ├── decentralized_coordinator.py # P2P right-of-way arbitration node
        │   ├── fleet_dashboard_server.py    # HTTP & SSE live dashboard server (port 8080)
        │   ├── generate_map.py              # Map generation utility
        │   ├── intent_broadcaster.py        # 5 Hz P2P intent publisher
        │   ├── navigate_choke_point.py      # Single-robot navigation test
        │   ├── task_allocator_cnp.py        # Contract Net Protocol task handoff node
        │   └── uncoordinated_choke_test.py  # Baseline uncoordinated test harness
        └── worlds/
            └── warehouse.sdf           # Gazebo Sim 12m x 12m warehouse world
```

---

## Build & Installation

### Prerequisites
- ROS 2 (Humble or compatible)
- Gazebo Sim (`ros_gz_sim`, `ros_gz_bridge`)
- Nav2 (`nav2_bringup`, `nav2_msgs`)
- Python 3

### Compiling the Workspace
```bash
cd /home/niket/amr_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

### 1. Launch Multi-Robot Simulation Stack
Launches Gazebo Sim, 3 AMRs (`amr1`, `amr2`, `amr3`), independent Nav2 stacks, P2P intent broadcasters, decentralized coordinators, battery simulators, CNP task allocators, and the fleet dashboard server:
```bash
source /home/niket/amr_ws/install/setup.bash
ros2 launch amr_simulation multi_robot_warehouse.launch.py
```

### 2. Access Live Fleet Web Dashboard
Open any web browser and navigate to:
```text
http://localhost:8080
```
- **Live 2D Map**: View robot movements, headings, and breadcrumb trails on the warehouse canvas in real-time.
- **Telemetry Cards**: Monitor individual AMR speed, $(x, y)$ coordinates, distance to the choke point, and battery SOC %.
- **CNP Activity Feed**: View live task handoff events, bidding calculations, and winner awards.
- **Interactive Controls**: Click buttons to dispatch missions or inject synthetic blockages.

### 3. Run Benchmark Evaluation Suite
To execute comparative benchmark trials and inspect the validation report:
```bash
source /home/niket/amr_ws/install/setup.bash
ros2 run amr_simulation benchmark_runner.py
```

### 4. Run Opposing Choke Point Coordination Test
To programmatically send conflicting head-to-head navigation goals:
```bash
source /home/niket/amr_ws/install/setup.bash
ros2 run amr_simulation coordinated_choke_test.py
```

---

## ROS 2 Topics & Interfaces

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/fleet/intent` | `amr_interfaces/msg/FleetIntent` | 5 Hz P2P broadcast of robot pose, velocity, planned waypoints, and state |
| `/fleet/task_auction` | `amr_interfaces/msg/TaskAuction` | CNP task auction announcement published by a blocked robot |
| `/fleet/task_bids` | `amr_interfaces/msg/TaskBid` | CNP bids submitted by candidate peer robots with heuristic cost scores |
| `/fleet/task_award` | `amr_interfaces/msg/TaskAward` | CNP task assignment announcement declaring the winning robot |
| `/{robot}/odom` | `nav_msgs/msg/Odometry` | Local odometry stream published per robot namespace |
| `/{robot}/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR ray sensor scan published per robot namespace |
| `/{robot}/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Motor velocity command topic per robot |
| `/{robot}/battery_state` | `sensor_msgs/msg/BatteryState` | 1 Hz dynamic battery state of charge (SOC) and voltage |
| `/{robot}/trigger_blockage`| `std_msgs/msg/Bool` | Trigger hook for simulated aisle blockage and task handoff testing |
| `/{robot}/navigate_to_pose`| `nav2_msgs/action/NavigateToPose` | Nav2 action server per robot namespace |
