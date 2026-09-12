The hardest Nav2 problems in ROS 2 AMR simulation are usually not “one bad parameter.” They are integration failures between **Gazebo, TF, time, sensors, localization, costmaps, lifecycle nodes, and controller physics**. These are also the errors where AI assistants often loop because the visible error is only a symptom, not the root cause.

## Most stubborn Nav2 errors

| Symptom or error | Hidden root cause | Why AI often loops | Correct investigation |
|---|---|---|---|
| `Timed out waiting for transform from base_link to map` | Missing or broken `map → odom → base_link` TF chain | It repeatedly suggests increasing `transform_tolerance` | Inspect every TF edge and identify which node should publish it |
| `Invalid frame ID "map"` | AMCL or SLAM is not publishing `map`, or localization has not started | It assumes the map server is the problem | Check whether you are using SLAM or AMCL, not both incorrectly |
| `Can't update static costmap layer, no map received` | Map topic, QoS, namespace, or `use_sim_time` mismatch | It recommends restarting Nav2 without checking topic delivery | Inspect `/map`, `/map_metadata`, QoS, and timestamps |
| Nav2 nodes remain `inactive` | Lifecycle manager cannot configure or activate one dependency | The first visible error may appear in lifecycle manager | Find the first node that failed, then inspect its own log |
| `Controller server failed to make progress` | Robot receives velocity commands but does not move enough | It blindly changes controller gains | Compare `/cmd_vel`, wheel/joint motion, `/odom`, and progress-checker thresholds |
| Robot spins forever | Bad localization, invalid path, wrong footprint, or controller unable to align | “Tune DWB” is suggested before proving localization | Verify pose, path, costmap, and command output in that order |
| Planner reports `NO_VALID_PATH` | Start or goal is occupied, unknown, outside the map, or blocked by inflation | It repeatedly changes planner plugins | Inspect the exact start/goal cells in global costmap |
| Robot refuses narrow doorways | Footprint or inflation radius makes the corridor mathematically impassable | It treats the issue as a planner failure | Compare robot footprint plus inflation against doorway width |
| Robot drives through obstacles | LiDAR frame, sensor topic, obstacle-layer configuration, or TF is wrong | It assumes the costmap is working because it is visible | Confirm obstacle marking and clearing independently |
| `use_sim_time` problems | Some nodes use Gazebo time while others use wall time | It suggests changing time settings randomly | Check `/clock` and `use_sim_time` for every relevant node |
| Parameters appear ignored | Wrong YAML nesting, wrong namespace, stale install overlay, or launch file loading another YAML | AI edits the YAML repeatedly | Print the live node parameters and verify the loaded file |
| `InvalidParameterTypeException` | YAML value is a string instead of a number, Boolean, list, or plugin declaration | YAML can look visually correct while being parsed differently | Query the live parameter type and rebuild/source the workspace |
| `Could not load library: libnav2_...so` | Plugin list belongs to a different Nav2 version or package is missing | It suggests writing a custom behavior tree unnecessarily | Check installed libraries, branch/distro compatibility, and plugin names |
| Planner crashes with `std::bad_alloc` | Excessive planner search space, huge lookup table, extreme map resolution, or invalid geometry | It gets misdiagnosed as an RMW/DDS problem | Reduce map size/resolution and inspect planner parameters |

Nav2’s own troubleshooting guide specifically emphasizes that missing `map` or `odom` frames usually means the simulation drivers/Gazebo are not active, the initial pose was not set, or lifecycle nodes were not activated. Costmap activation can block until the required TF tree exists. [docs.nav2](https://docs.nav2.org/development_guides/build_docs/build_troubleshooting_guide.html)

## 1. TF errors: the biggest root cause

For a standard AMR, the expected chain is:

```text
map → odom → base_link → laser_link
```

Usually:

- `map → odom` comes from AMCL or SLAM Toolbox.
- `odom → base_link` comes from wheel odometry or a Gazebo odometry plugin.
- `base_link → laser_link` comes from URDF and `robot_state_publisher`.
- `/scan` must use the same laser frame named in its message header.

Check the tree:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

Check the scan frame:

```bash
ros2 topic echo /scan --once
```

Look at:

```yaml
header:
  frame_id: laser_link
```

A common failure is that the robot publishes `base_footprint`, while Nav2 is configured with `base_link`, or the LiDAR publishes `lidar`, while the URDF uses `laser_link`. Increasing TF timeouts does not fix a missing frame; it only makes Nav2 wait longer.

### AI-loop pattern

A typical unhelpful loop is:

1. “Increase `transform_tolerance`.”
2. Error remains.
3. “Increase `transform_timeout`.”
4. Error remains.
5. “Restart Gazebo.”

The correct question is:

> Which node is supposed to publish the missing transform, and is that node actually publishing it?

## 2. Simulation time and `/clock`

In Gazebo simulation, Nav2, SLAM Toolbox, robot state publisher, sensor drivers, and localization nodes should generally use simulation time consistently:

```yaml
use_sim_time: true
```

Verify that Gazebo publishes the clock:

```bash
ros2 topic echo /clock --once
ros2 topic hz /clock
```

Inspect node parameters:

```bash
ros2 param get /controller_server use_sim_time
ros2 param get /planner_server use_sim_time
ros2 param get /bt_navigator use_sim_time
ros2 param get /slam_toolbox use_sim_time
ros2 param get /robot_state_publisher use_sim_time
```

If one node uses wall time and another uses simulation time, you may see:

- TF extrapolation into the past or future.
- Messages rejected as too old.
- Costmaps that stop updating.
- SLAM appearing frozen.
- Nav2 actions timing out.
- A robot that works only after pausing and restarting Gazebo.

Do not solve this by randomly increasing `transform_tolerance`. First make time consistent.

## 3. Lifecycle manager failures

Nav2 uses lifecycle nodes. A node can be running as a process but still not be active.

Inspect lifecycle states:

```bash
ros2 lifecycle nodes
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /local_costmap/local_costmap
ros2 lifecycle get /global_costmap/global_costmap
```

If bringup aborts, find the **first crashed or failed node**, not the final message:

```text
lifecycle_manager_navigation:
  Failed to bring up all requested nodes
```

That message is normally a consequence. The useful error is earlier, such as:

```text
InvalidParameterTypeException
Could not load library
std::bad_alloc
Failed to configure
```

The Nav2 troubleshooting documentation also recommends checking that lifecycle nodes are activated when transforms or servers appear to be missing. [docs.nav2](https://docs.nav2.org/development_guides/build_docs/build_troubleshooting_guide.html)

## 4. YAML and parameter errors

One of the most stubborn categories is when the YAML file is edited correctly but Nav2 still reports the old value. A Nav2 issue demonstrates this exact pattern: `footprint_padding` was interpreted as a string instead of a double, causing planner and controller processes to terminate during startup. After rebuilding, the same issue thread also showed a planner memory failure and a missing behavior-tree library. [github](https://github.com/ros-navigation/navigation2/issues/3703)

Correct:

```yaml
footprint_padding: 0.05
use_sim_time: true
enabled: true
inflation_radius: 0.5
```

Potentially problematic:

```yaml
footprint_padding: "0.05"
use_sim_time: "true"
enabled: "true"
```

For a footprint, use a valid YAML list or a supported string format for your Nav2 version. Do not assume quotation marks are harmless.

Verify what Nav2 actually loaded:

```bash
ros2 param dump /global_costmap/global_costmap
ros2 param dump /local_costmap/local_costmap
ros2 param get /global_costmap/global_costmap footprint_padding
```

If the old value appears:

1. Confirm the launch file points to the intended YAML.
2. Rebuild the workspace.
3. Source the correct overlay.
4. Open a new terminal.
5. Check for duplicate packages or stale installs.

Example:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix your_navigation_package
```

A frequent mistake is editing:

```text
src/your_package/config/nav2_params.yaml
```

while the launch file is actually loading:

```text
install/your_package/share/your_package/config/nav2_params.yaml
```

## 5. Planner failures and `std::bad_alloc`

Planner failures are often blamed on the wrong component. A message such as:

```text
No valid path could be found
```

can be caused by:

- The start pose being inside an obstacle.
- The goal pose being occupied.
- Excessive inflation.
- Unknown space being disallowed.
- A map that does not correspond to the Gazebo world.
- An incorrect global frame.
- A robot footprint larger than expected.

For Smac Hybrid-A*, additional causes include:

- Unrealistic `minimum_turning_radius`.
- A planner configured for a car-like robot when the AMR is differential drive.
- Excessive map resolution.
- Very large search bounds.
- An enormous lookup table.
- A malformed or huge map.

A `std::bad_alloc` during planner configuration can result from excessive memory requirements. In the cited issue, the planner reported an unusually large heuristic lookup table immediately before the memory allocation failure. [github](https://github.com/ros-navigation/navigation2/issues/3703)

For a normal differential-drive AMR, start with a simpler planner:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

Once the complete pipeline works, test Smac Planner separately. Do not debug localization, costmaps, and a complex planner simultaneously.

## 6. Costmap problems

Costmaps are where sensor, TF, map, footprint, and time problems converge.

Inspect the topics:

```bash
ros2 topic list | grep costmap
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
ros2 topic echo /global_costmap/published_footprint --once
```

In RViz, display:

- Global costmap.
- Local costmap.
- LaserScan.
- Robot footprint.
- Global plan.
- Local plan.
- TF.
- Robot model.

### Static costmap failures

Typical causes:

- No `/map` topic.
- Map server not active.
- SLAM Toolbox is being used but Nav2 expects a static map.
- QoS mismatch.
- Wrong namespace.
- `map → odom` is absent.
- Map frame is not named `map`.

### Obstacle-layer failures

Typical causes:

- Wrong scan topic.
- Wrong sensor frame.
- Laser mounted outside the TF tree.
- Incorrect `observation_sources`.
- `marking` or `clearing` disabled.
- Scan range does not match the simulation.
- Sensor data uses wall time.
- LiDAR is publishing `PointCloud2` while configured as `LaserScan`.

Check:

```bash
ros2 topic info /scan -v
ros2 topic hz /scan
ros2 topic echo /scan --once
```

A visible LiDAR display in RViz does not prove the obstacle layer is receiving usable data.

## 7. Navigation versus robot motion

A very important diagnostic split is:

### Nav2 publishes no velocity

Investigate:

- Lifecycle state.
- BT Navigator.
- Planner result.
- Controller server.
- Goal validity.
- Costmap validity.
- Controller plugin configuration.

Commands:

```bash
ros2 topic hz /cmd_vel
ros2 action list
ros2 topic echo /cmd_vel
```

### Nav2 publishes velocity but robot does not move

Investigate:

- Gazebo command topic.
- Topic remapping.
- Differential-drive plugin.
- Wheel joint names.
- Joint limits.
- Robot friction.
- Velocity command timeout.
- Command mux or safety node.
- Whether the simulator expects `Twist` or `TwistStamped`.

For example, Nav2 may publish:

```text
/cmd_vel
```

while the Gazebo plugin listens to:

```text
/robot/cmd_vel
```

Then use an explicit remap in the launch file:

```python
remappings=[
    ('cmd_vel', '/robot/cmd_vel')
]
```

Do not remap blindly. First inspect both topics:

```bash
ros2 topic list | grep cmd_vel
ros2 topic info /cmd_vel
ros2 topic info /robot/cmd_vel
```

If Nav2 publishes commands and the robot moves in Gazebo but `/odom` does not change, the odometry integration is broken. Nav2 will eventually report that the robot failed to make progress.

## 8. Robot oscillation and spinning

A robot that rotates endlessly may have several different causes:

- AMCL pose is incorrect.
- The goal orientation is impossible or badly chosen.
- The local controller cannot find a valid trajectory.
- The footprint overlaps an obstacle.
- `min_theta_velocity_threshold` is too high.
- The robot’s simulated angular velocity is too low.
- The robot is commanded to rotate but odometry does not reflect it.
- DWB critic weights favor alignment over forward movement.
- The local costmap is stale.

Separate the cases:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

If `/cmd_vel` shows angular velocity but `/odom` yaw does not change, it is a simulator or drive-plugin problem.

If both change but the controller keeps rotating, inspect:

- The local plan.
- Current localization pose.
- Goal orientation.
- Costmap obstacles.
- Goal checker tolerances.

For initial testing, use a relaxed goal:

```yaml
goal_checker:
  xy_goal_tolerance: 0.25
  yaw_goal_tolerance: 0.5
```

Do not permanently loosen tolerances to hide a localization or odometry problem.

## 9. SLAM Toolbox versus AMCL confusion

For mapping a new simulated environment:

```text
Gazebo sensors → SLAM Toolbox → map → Nav2
```

For navigating using an existing map:

```text
Map server + AMCL → map → Nav2
```

Avoid launching AMCL and SLAM Toolbox as competing publishers of `map → odom`. That can create unstable or conflicting transforms.

### Mapping mode

Use SLAM Toolbox and do not require a pre-existing map for navigation. The robot still needs:

- Valid odometry.
- Valid laser TF.
- `/scan`.
- A working `odom → base_link`.
- Correct simulation time.
- Teleoperation or autonomous motion.

### Localization mode

Use:

- Map server.
- AMCL.
- Existing map YAML.
- Initial pose in RViz.

The Nav2 documentation explicitly notes setting an initial pose when map or odom frames are missing at startup. [docs.nav2](https://docs.nav2.org/development_guides/build_docs/build_troubleshooting_guide.html)

## 10. Errors AI commonly fails to solve

These are especially good candidates for a troubleshooting dataset or an AI assistant benchmark because they require evidence from multiple ROS 2 components.

### “The YAML is correct, but the error remains”

Likely causes:

- Wrong file loaded.
- Stale install directory.
- Incorrect namespace nesting.
- Duplicate package in the workspace.
- Parameter type conversion.
- Another launch file overwriting the value.

Required evidence:

```bash
ros2 param dump /node_name
ros2 pkg prefix package_name
```

### “TF exists, but Nav2 still times out”

Likely causes:

- TF exists under a different frame name.
- Timestamp is invalid.
- Transform is published on a different namespace.
- Static transform is missing.
- `use_sim_time` mismatch.
- Transform is available only intermittently.

Required evidence:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

### “The costmap looks empty”

Likely causes:

- Sensor data is not reaching the obstacle layer.
- Scan frame cannot transform into the costmap frame.
- Observation source name does not match its configuration.
- Costmap is not active.
- Sensor range filtering removes all readings.
- RViz is displaying the wrong namespace.

### “AI keeps suggesting parameter tuning”

This usually happens because the diagnosis starts at the controller instead of the data pipeline. The correct order is:

```text
Clock → topics → TF → localization → costmaps → planner → controller → simulator motion
```

Changing planner or controller parameters before checking those layers creates a tuning loop.

## A reliable debugging procedure

Use this order for almost every AMR simulation:

1. **Confirm the ROS distribution and overlay.**

   ```bash
   printenv | grep -i ROS
   ros2 doctor --report
   ```

2. **Confirm Gazebo and simulation time.**

   ```bash
   ros2 topic hz /clock
   ```

3. **Confirm sensor data.**

   ```bash
   ros2 topic hz /scan
   ros2 topic echo /odom --once
   ```

4. **Confirm the TF chain.**

   ```bash
   ros2 run tf2_tools view_frames
   ros2 run tf2_ros tf2_echo odom base_link
   ros2 run tf2_ros tf2_echo base_link laser_link
   ```

5. **Confirm localization.**

   ```bash
   ros2 topic echo /amcl_pose --once
   ros2 topic echo /pose --once
   ```

6. **Confirm lifecycle states.**

   ```bash
   ros2 lifecycle nodes
   ros2 lifecycle get /controller_server
   ```

7. **Confirm costmap updates.**

   ```bash
   ros2 topic hz /local_costmap/costmap
   ros2 topic hz /global_costmap/costmap
   ```

8. **Confirm planner output.**

   Check whether the global plan appears in RViz and whether the goal is in free space.

9. **Confirm controller output.**

   ```bash
   ros2 topic hz /cmd_vel
   ros2 topic echo /cmd_vel
   ```

10. **Confirm physical simulation response.**

   Check wheel joints, Gazebo motion, and `/odom` simultaneously.

## Strong research topics for your project

If your goal is to study “hard-to-fix Nav2 errors,” these are the most valuable categories:

- TF tree failures caused by inconsistent frame names.
- Simulation-time versus wall-time synchronization.
- Stale YAML and incorrect launch-file parameter loading.
- Lifecycle bringup failures where the final error hides the first crash.
- Costmap failures caused by QoS and sensor-frame mismatches.
- Planner memory exhaustion from map resolution and search parameters.
- DWB or MPPI oscillation caused by invalid odometry feedback.
- AMCL localization failure in visually or geometrically repetitive environments.
- SLAM Toolbox and AMCL both publishing conflicting transforms.
- Namespaces and multi-robot remapping failures.
- Gazebo `/cmd_vel` mismatch and silent command drops.
- Robot footprint and inflation making valid paths impossible.
- DDS/RMW shared-memory errors that look like Nav2 failures but are middleware or stale-process problems.

The central lesson is: **do not tune Nav2 until the data path is proven**. For an AMR, the minimum proof is valid `/clock`, `/scan`, `/odom`, a complete TF chain, active lifecycle nodes, updating costmaps, and verified `/cmd_vel`-to-motion behavior.