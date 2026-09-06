# Edge-AI AMR Fleet Coordination: Implementation Protocol

> [!INFO] Architecture Overview
> 
> This protocol details a fully decentralized multi-agent robotic system running on ROS 2 and Nav2. All coordination occurs peer-to-peer via Data Distribution Service (DDS), substituting central fleet servers with onboard edge-AI inference.
> 
>   

## Phase 1: Simulation World & Single-Robot Baseline

### 1.1 Gazebo World Configuration

Construct a Gazebo environment enforcing strict dimensional bottlenecks.

  

- Create `warehouse.sdf` with static warehouse racks.
    
      
    
- Enforce standard aisle widths of $2.5\text{ m}$.
    
      
    
- Construct a central choke point or 4-way intersection with a clearance of $1.1\text{ m}$ to $1.2\text{ m}$. (This allows one $0.6\text{ m}$ wide AMR to pass but prevents simultaneous two-way traffic).
    
      
    
- Configure high ground friction in the SDF to prevent odometric drift:
    
      
    
    XML
    
    ```
    <collision name="collision">
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    ```
    

### 1.2 Robot URDF & Sensor Plugins

Build the AMR description (`amr.xacro`) with differential drive and noisy sensors.

  

- Attach the `libgazebo_ros_ray_sensor.so` plugin to the LiDAR link.
    
      
    
- Configure the LiDAR for $10\text{ Hz}$ update rate, $360^\circ$ FOV, and $10.0\text{ m}$ range.
    
      
    
- Inject Gaussian noise into the LiDAR to replicate physical hardware:
    
      
    
    XML
    
    ```
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
    ```
    
- Attach `libgazebo_ros_diff_drive.so` to publish `/odom` and map the `odom` $\to$ `base_footprint` TF tree.
    
      
    

### 1.3 Nav2 Bringup

Configure the single-robot navigation stack.

  

- **Global Costmap**: `map_server` (static map) + `inflation_layer` (radius: $0.45\text{ m}$, cost scaling: $3.0$).
    
      
    
- **Local Costmap**: $3.0\text{ m} \times 3.0\text{ m}$ rolling window ($0.05\text{ m}$ resolution).
    
      
    
- **Controller**: `RegulatedPurePursuitController` (Max linear: $0.8\text{ m/s}$, Max angular: $1.0\text{ rad/s}$).
    
      
    
- **Verification**: Run a programmatic `ActionClient` sending `/navigate_to_pose` goals through the choke point to confirm unattended operation.
    
      
    

## Phase 2: Multi-Robot Spawn & Uncoordinated Baseline

### 2.1 Namespace Isolation

Create `multi_robot_bringup.launch.py` to spawn $\ge3$ AMRs.

  

- Isolate every node using `PushRosNamespace` (e.g., `/robot1`, `/robot2`, `/robot3`).
    
      
    
- Remap all internal topics and TF frames. If using `robot_state_publisher`, pass the `frame_prefix` parameter (`robot1/`) so TF trees are fully isolated (`robot1/odom` $\to$ `robot1/base_link`).
    
      
    
- Spawn coordinates:
    
      
    - `/robot1`: `x: -4.0, y: 0.0, yaw: 0.0`
        
          
        
    - `/robot2`: `x: 4.0, y: 0.0, yaw: 3.14`
        
          
        
    - `/robot3`: `x: 0.0, y: -4.0, yaw: 1.57`
        
          
        

### 2.2 Independent Nav2 Stacks

- Launch an independent `nav2_bringup` instance per namespace.
    
      
    
- Verify that `/robot1/local_costmap` only subscribes to `/robot1/scan`.
    
      
    
- Verify that each robot is running its own autonomous brain rather than sharing a planner.
    
      
    

### 2.3 Baseline Logging (The Control Condition)

- Disable all peer coordination.
    
      
    
- Write a Python script to dispatch intersecting goals that force all three robots into the choke point simultaneously.
    
      
    
- Log the timestamp from first goal dispatch to total failure/completion. This documents the default Nav2 "stop-and-wait" behavior and provides the Phase 7 comparison benchmark.
    
      
    

## Phase 3: Decentralized Peer-to-Peer Intent Sharing

> [!WARNING] No Brokers Do not use an MQTT broker or central ROS master. Use native ROS 2 DDS multicast for flat mesh communication.
> 
>   

### 3.1 Custom Interface Definition

Define `FleetIntent.msg` in a custom `amr_interfaces` package.

  

YAML

```
string robot_id
builtin_interfaces/Time stamp
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_velocity
geometry_msgs/Pose[] planned_waypoints
uint8 current_state
```

### 3.2 Extractor & Broadcaster Node

Write `intent_broadcaster.py` running locally on each robot.

  

- **Odometry Extraction**: Subscribe to local `/robotN/odom` for `current_pose` and `current_velocity`.
    
      
    
- **Path Downsampling**: Subscribe to the Nav2 global plan (`/robotN/plan`). Parse the dense trajectory array and extract 3 waypoints representing future locations at $t+1.0\text{s}$, $t+2.0\text{s}$, and $t+3.0\text{s}$.
    
      
    
- **Publish Trigger**: Publish the assembled `FleetIntent` to the global `/fleet/intent` topic at a fixed $5\text{ Hz}$ heartbeat.
    
      
    

### 3.3 QoS and P2P Reception

- Subscribe to `/fleet/intent` on all robots.
    
      
    
- Set Quality of Service (QoS) to `BEST_EFFORT` reliability and `VOLATILE` durability to prevent network queuing if packets drop.
    
      
    
- Maintain a local state dictionary: `self.peer_intents[msg.robot_id] = msg`.
    
      
    

## Phase 4: Conflict Detection & Negotiation

### 4.1 Spatio-Temporal Collision Check

Run at $10\text{ Hz}$ inside the local control loop.

  

- Extract self trajectory $P_i(t)$ and peer trajectory $P_j(t)$.
    
      
    
- Calculate Euclidean distance at synchronized time steps:
    
      
    
    $$d_{ij}(t_k) = \sqrt{(x_i(t_k) - x_j(t_k))^2 + (y_i(t_k) - y_j(t_k))^2}$$
    
- If $d_{ij} < 1.2\text{ m}$ (robot radii + safety margin), set `conflict_detected = True`.
    
      
    

### 4.2 Deterministic Arbitration (Fallback)

Implement a hardcoded tie-breaker for basic deadlock resolution.

  

- **Rule**: Compare $t_k$ for both robots reaching the conflict zone. If $\Delta t < 0.3\text{ s}$, compare string IDs. Lowest ID proceeds, highest ID yields.
    
      
    
- **Yielding Execution**: Yielding AMR overrides `/cmd_vel` with a zero-twist command or cancels the Nav2 goal temporarily.
    
      
    

### 4.3 Edge-AI Inference (MLP)

Train and deploy a Multi-Layer Perceptron (MLP) to replace the deterministic rule.

  

- **Input Vector ($X \in \mathbb{R}^{10}$)**: $[\Delta x, \Delta y, v_{x,i}, v_{y,i}, v_{x,j}, v_{y,j}, \theta_i, \theta_j, d_{\text{choke},i}, d_{\text{choke},j}]$
    
      
    
- **Architecture**: Linear(10, 32) $\to$ ReLU $\to$ Linear(32, 16) $\to$ ReLU $\to$ Linear(16, 2) $\to$ Softmax.
    
      
    
- **Training**: Imitation learning based on $1000$ simulated deterministic conflict scenarios.
    
      
    
- **Inference**: Export to ONNX (`model.onnx`). Use Python `onnxruntime` inside the ROS 2 node to calculate `[Yield, Proceed]` probabilities dynamically at the edge.
    
      
    

## Phase 5: Blocked-Aisle Task Rerouting (Contract Net Protocol)

### 5.1 Blockage Detection

- Monitor Nav2 controller state.
    
      
    
- If linear velocity is $< 0.05\text{ m/s}$ for $>3.0\text{ s}$ despite a clear target, or the local planner fails continuously, trigger `TASK_BLOCKED`.
    
      
    

### 5.2 Task Auction (CNP)

- Blocked AMR publishes `TaskAuction.msg` on `/fleet/task_auction`.
    
      
    
- Idle/available peers evaluate the auction by calculating a local cost:
    
      
    
    $$\text{Cost} = (1.0 \times \text{Distance}) + (0.5 \times (100 - \text{Battery}))$$
    
- Peers broadcast `TaskBid.msg` to `/fleet/task_bids`.
    
      
    
- After a $500\text{ ms}$ timeout, the lowest bid claims the task.
    
      
    
- The original blocked AMR reroutes to standby without requiring a central dispatcher.
    
      
    

## Phase 6: Fleet Dashboard

### 6.1 WebSocket Bridge

Expose ROS 2 topics to standard web traffic.

  

Bash

```
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

### 6.2 Frontend Architecture (HTML5/roslibjs)

Do not build a backend API. Connect the frontend directly to the WebSocket.

  

JavaScript

```
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

const odomSub = new ROSLIB.Topic({
  ros: ros,
  name: '/robot1/odom',
  messageType: 'nav_msgs/Odometry'
});

odomSub.subscribe((msg) => {
  renderRobotPosition('robot1', msg.pose.pose.position.x, msg.pose.pose.position.y);
});
```

- Render positions on an HTML5 canvas overlaid on the warehouse map.
    
      
    
- Subscribe to synthetic `/robotN/battery_state` and `/robotN/status` topics to render UI badges.
    
      
    

## Phase 7: Benchmark & Validation

### 7.1 Automated Testing Harness

- Write `benchmark.py` to run 10 identical loop iterations of the bottleneck scenario.
    
      
    
- **Condition A**: Phase 2 uncoordinated baseline.
    
      
    
- **Condition B**: Phase 4/5 full decentralized stack.
    
      
    

### 7.2 Core Metrics

Log and output the following for validation against success criteria:

  

1. **Total Fleet Time**: $t_{\text{finish}} - t_{\text{start}}$. (Target: $\ge20\%$ reduction from Condition A).
    
      
    
2. **Collisions**: Count of events where $\Vert{}\mathbf{p}_i - \mathbf{p}_j\Vert{} < 0.5\text{ m}$. (Target: $0$).
    
      
    
3. **Negotiation Latency**: Sub-millisecond calculation via ONNX execution profiling.




# Phase  distribution: 
### Plan Verification: Is this aligned with the Guide?

The implementation plan is aligned with the stages outlined in your prototype build guide:

* **Work Package 1 (Multi-Robot Sim)** implements **Phase 2** (namespacing $\ge 3$ robots under `/amr1`, `/amr2`, `/amr3`, independent Nav2 stacks, and parameterizing URDF/Xacro).


* **Work Package 2 (Uncoordinated Baseline)** fulfills the control condition requirement of **Phase 2 & Phase 7** (running uncoordinated conflicting routes, inducing deadlocks, and logging the baseline timestamp).


* **Work Package 3 (Coordination Layer)** addresses **Phase 3 & Phase 4** (peer-to-peer `/fleet/intent` sharing over DDS, spatial conflict checks, and tie-breaking right-of-way).


* **Work Package 4 (Battery & Dynamic Tasks)** covers **Phase 5 & Phase 6** (synthetic battery drain, Contract Net Protocol task handoff, dynamic rerouting).


* **Work Package 5 & 6 (Benchmarking & Dashboard)** directly map to **Phase 6 & Phase 7** (rosbridge + web/canvas dashboard, automated trials to prove $\ge 20\%$ time reduction and zero collisions).



---

### Should you divide this goal into multiple parts?

**Yes, you must divide it.**

Executing this plan in a single monolithic prompt will cause the coding agent to generate hundreds of lines of boilerplate across 8+ files simultaneously (Xacro macros, multiple Nav2 parameter blocks, ROS-Gazebo bridges, coordinate transforms, and Python nodes). If a single TF prefix or bridge mapping breaks, debugging three simultaneous Nav2 stacks at once becomes difficult.

---

### Recommended Execution Breakdown

Divide the plan into **4 manageable execution sprints**:

#### Sprint 1: Multi-Robot Spawning & Isolated Nav2 Bringup (WP1)

* **Goal**: Parameterize `amr.xacro` with `<xacro:arg name="prefix" ... />` and get 3 robots spawning in Gazebo with isolated namespaces (`amr1`, `amr2`, `amr3`).


* **Deliverable**: Verify that all 3 robots can independently accept a Nav2 navigation goal without cross-talk or TF collisions.


* **Why split here**: Namespacing Nav2, static transforms (`map -> <robot>/odom`), and Gazebo topic remappings is error-prone. You need this verified before adding any coordination code.

#### Sprint 2: The Uncoordinated Baseline Benchmark (WP2 & WP5 Setup)

* **Goal**: Build `uncoordinated_choke_test.py` and the basic CSV logger.


* **Deliverable**: Run `amr1` and `amr2` head-to-head into the 1.15 m choke point simultaneously. Document the resulting deadlock/livelock and export the baseline completion time / failure rate to `benchmark_results.csv`.


* **Why split here**: This locks in your control condition (Condition A) for Phase 7 before any smart logic is written.



#### Sprint 3: Decentralized P2P Coordination & Choke-Point Reservation (WP3)

* **Goal**: Implement `decentralized_coordinator.py` on each robot.


* **Deliverable**: Broadcast `/fleet/intent`, detect spatial overlaps before entering the 1.15 m corridor, route yielding robots to holding points, and release reservation on exit. Run the same head-to-head test and prove zero collisions/deadlocks.



#### Sprint 4: Fleet Dashboard, Battery Drain & Final Benchmarks (WP4 & WP6)

* **Goal**: Add `battery_simulator.py`, blocked-aisle re-routing (Phase 5), the web dashboard (rosbridge + HTML/Canvas), and run the automated 10-trial comparison script.


* **Deliverable**: Verify the final $\ge 20\%$ time reduction figure and capture dashboard visuals for your SIH presentation.



---

### Answers to the Implementation Plan's Open Questions

1. **Number of Robots**: **Stick to 3 robots**.


* Two opposing robots (`amr1` and `amr2`) traversing the narrow 1.15 m choke point in opposite directions, plus one robot (`amr3`) operating cross-traffic or acting as the recipient for task handoff.


* Running 4–5 robots with individual Nav2 stacks and Gazebo GPU LiDARs will cause physics simulation slowdowns on laptop hardware. Three robots satisfy all problem statement requirements while keeping compute overhead manageable.




2. **Fleet Dashboard**: **Lightweight Web Dashboard (`rosbridge_websocket` + HTML5/Canvas/roslibjs)**.


* It runs directly in any browser without installing desktop Qt dependencies.


* It directly matches the Phase 6 guidance and looks cleaner for live hackathon project presentations.