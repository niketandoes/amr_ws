TF errors in Nav2 usually mean that one required coordinate-frame relationship is missing, disconnected, duplicated, incorrectly named, or published with bad timestamps. For a typical AMR, Nav2 needs this connected chain:

```text
map → odom → base_link → sensor_link
```

For example, the LiDAR may use `laser_link`:

```text
map → odom → base_link → laser_link
```

Nav2’s setup guide identifies these three required relationships: `map → odom`, `odom → base_link`, and `base_link →` each sensor frame. [docs.nav2](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/)

## What TF actually does

A TF frame is a coordinate system. It tells ROS 2 where something is and how it is oriented relative to another frame.

For an AMR:

- `map` is the globally consistent world frame.
- `odom` is the locally smooth motion frame, usually from wheel odometry.
- `base_link` is attached to the robot body.
- `laser_link`, `lidar_link`, or `camera_link` is attached to a sensor.
- `base_footprint` is sometimes used as a ground-projected robot frame.

TF is not usually a single transform published by one node. Different parts of the chain come from different sources:

| Transform | Typical publisher | Type |
|---|---|---|
| `map → odom` | AMCL or SLAM Toolbox | Dynamic |
| `odom → base_link` | Gazebo odometry plugin, wheel odometry, or `robot_localization` | Dynamic |
| `base_link → laser_link` | `robot_state_publisher` from URDF | Static |
| `base_link → camera_link` | `robot_state_publisher` from URDF | Static |
| `base_link → wheel_link` | `robot_state_publisher` from URDF/joint states | Usually dynamic or static depending on joint |

`robot_state_publisher` reads the URDF and publishes the poses of the robot’s links to TF2. [docs.ros](https://docs.ros.org/en/ros2_packages/rolling/api/robot_state_publisher/)

## The three main TF failures

### 1. Missing frame

Typical message:

```text
Invalid frame ID "map" passed to canTransform
target_frame - frame does not exist
```

or:

```text
Frame [base_link] does not exist
```

This means TF2 has never received that frame from any broadcaster.

Common causes:

- AMCL or SLAM Toolbox is not running.
- No initial pose has been sent to AMCL.
- Gazebo is not publishing odometry.
- `robot_state_publisher` is not running.
- The URDF was not loaded.
- The frame is named `base_footprint`, but Nav2 expects `base_link`.
- The LiDAR frame is named `lidar`, but the scan message says `laser_link`.
- A namespace was added to one part of the system but not another.

A missing frame is not fixed by increasing timeout values. You must find the node that should publish it.

### 2. Disconnected tree

You may see both groups:

```text
map
odom
```

and separately:

```text
base_link
laser_link
```

Each group exists, but there is no path connecting them. Nav2 still cannot navigate because it needs a complete chain from the global frame to the robot and sensors.

This often happens when:

- `odom → base_link` is absent.
- `base_link → laser_link` is absent.
- A static transform uses a different spelling.
- One node publishes `robot/base_link`, while another expects `base_link`.

### 3. Timestamp or extrapolation failure

Typical messages:

```text
Lookup would require extrapolation into the future
```

```text
Lookup would require extrapolation into the past
```

```text
TF_OLD_DATA ignoring data from the past
```

These usually indicate inconsistent clocks, delayed messages, or transforms being published at incompatible times. ROS 2’s TF debugging tools specifically support inspecting transform timing and broadcaster delays with `tf2_monitor`. [docs.ros](https://docs.ros.org/en/rolling/p/tf2_ros/doc/cli_tools.html)

Common causes:

- Gazebo uses simulation time, but Nav2 uses wall time.
- `/clock` is missing or paused.
- Some nodes have `use_sim_time: true`, others have `false`.
- A simulator was restarted while old nodes were still running.
- System time changed.
- Sensor timestamps are stale.
- A transform is being published too slowly.
- A bag or simulation is being replayed at an unusual rate.

## First diagnostic: generate the TF tree

Start the simulation and Nav2, then run:

```bash
ros2 run tf2_tools view_frames
```

This generates a TF graph file in the current directory. The tool also reports useful timing information, including broadcaster, average rate, buffer length, most recent transform, and oldest transform. [docs.ros](https://docs.ros.org/en/rolling/p/tf2_ros/doc/cli_tools.html)

Inspect the graph for:

- Missing `map`.
- Missing `odom`.
- Missing `base_link`.
- Sensor frames not connected to the robot.
- Two different root frames.
- Unexpected names such as `robot1/base_link`.
- Multiple broadcasters publishing the same parent-child pair.
- A transform with a suspiciously low rate.
- Very old or delayed transforms.

A healthy single-robot tree generally resembles:

```text
map
└── odom
    └── base_link
        ├── base_footprint
        ├── laser_link
        ├── imu_link
        ├── camera_link
        ├── left_wheel_link
        └── right_wheel_link
```

The exact tree can vary, but there should normally be one connected tree for the robot.

## Second diagnostic: test each link directly

Do not immediately test only `map` to `base_link`. Test the chain one edge at a time.

### Check localization

```bash
ros2 run tf2_ros tf2_echo map odom
```

Expected result:

```text
At time ...
- Translation: [...]
- Rotation: ...
```

If this fails:

- With AMCL, set the initial pose in RViz.
- Check that the map server is active.
- Check that AMCL is active.
- Confirm AMCL receives `/scan`, `/map`, and the robot pose.
- With SLAM Toolbox, confirm SLAM is running and publishing the map-to-odom relationship.

### Check odometry

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

If this fails:

- Check `/odom`.
- Check the Gazebo drive plugin.
- Check wheel joint names.
- Check the odometry plugin’s configured `frame_id`.
- Check whether it publishes `base_link` or `base_footprint`.
- Check topic remapping and namespaces.

### Check the sensor transform

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

Replace `laser_link` with the actual frame in your scan message.

If this fails:

- Inspect the scan message:

  ```bash
  ros2 topic echo /scan --once
  ```

- Read its `header.frame_id`.
- Check that the same frame exists in the URDF.
- Check that the sensor link is connected to `base_link`.
- Confirm `robot_state_publisher` is running.

The Nav2 documentation demonstrates the same method with `tf2_echo base_link base_laser` and recommends URDF plus `robot_state_publisher` for real robot projects rather than relying on manually launched static publishers. [docs.nav2](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/)

## Third diagnostic: inspect the message frame

A very common error is debugging the wrong sensor frame.

Run:

```bash
ros2 topic echo /scan --once
```

Look for:

```yaml
header:
  frame_id: laser
```

Then test:

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

Not:

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

unless the message actually uses `laser_link`.

The names must match exactly. These are different frame IDs:

```text
laser
laser_link
Laser
robot/laser
/laser
```

Depending on the ROS 2 and Nav2 configuration, leading slashes and namespaces can also create confusing mismatches. Pick one naming convention and use it consistently.

## Fourth diagnostic: inspect TF topics

TF data is normally published on:

```text
/tf
/tf_static
```

Check them:

```bash
ros2 topic list | grep tf
ros2 topic info /tf -v
ros2 topic info /tf_static -v
```

To inspect publishers:

```bash
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

You can also inspect messages:

```bash
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

Look at each transform’s:

```yaml
header:
  frame_id: parent_frame
child_frame_id: child_frame
```

For example:

```yaml
header:
  frame_id: base_link
child_frame_id: laser_link
```

This means the transform is:

```text
base_link → laser_link
```

A common mistake is accidentally publishing the reverse relationship or using the sensor as the parent.

## Fifth diagnostic: check time

For Gazebo simulation, verify `/clock`:

```bash
ros2 topic echo /clock --once
ros2 topic hz /clock
```

Then inspect Nav2:

```bash
ros2 param get /planner_server use_sim_time
ros2 param get /controller_server use_sim_time
ros2 param get /bt_navigator use_sim_time
ros2 param get /global_costmap/global_costmap use_sim_time
ros2 param get /local_costmap/local_costmap use_sim_time
```

Also inspect localization and robot-state nodes:

```bash
ros2 param get /amcl use_sim_time
ros2 param get /slam_toolbox use_sim_time
ros2 param get /robot_state_publisher use_sim_time
```

For a simulation, they should normally agree. A frequent failure is:

```text
Gazebo: simulation time
Nav2: system time
AMCL: system time
```

or:

```text
Nav2: simulation time
robot_state_publisher: system time
```

That produces apparently irrational TF behavior: the frame exists, but Nav2 says it is too old or in the future.

Restart the entire system after changing time configuration. Do not leave old Nav2, Gazebo, or localization processes running:

```bash
pkill -f rviz2
pkill -f nav2
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
```

Use caution with broad `pkill` commands on a machine running other robotics processes.

## Who should publish each transform?

### `map → odom`

For AMCL:

```text
map_server + AMCL → map → odom
```

For SLAM:

```text
SLAM Toolbox → map → odom
```

Do not normally publish this manually with a static transform. It represents the changing correction between global localization and local odometry.

If both AMCL and SLAM Toolbox publish it, you can get competing transforms and unstable localization.

Check likely broadcasters using:

```bash
ros2 run tf2_tools view_frames
```

The generated graph identifies the broadcaster for each frame. [docs.ros](https://docs.ros.org/en/rolling/p/tf2_ros/doc/cli_tools.html)

### `odom → base_link`

This must come from the odometry system. In simulation, it is commonly supplied by the differential-drive or other mobile-base plugin.

Check whether odometry changes when the robot moves:

```bash
ros2 topic echo /odom
```

If the robot physically moves in Gazebo but `odom` remains unchanged, Nav2 cannot track progress correctly.

If odometry uses `base_footprint` instead:

```text
odom → base_footprint
```

then configure Nav2 consistently or publish the expected relationship. Do not simply create a second unrelated odometry transform.

### `base_link → sensor_link`

This is normally a fixed transform from the URDF, published by `robot_state_publisher`.

Example URDF structure:

```xml
<link name="base_link"/>
<link name="laser_link"/>

<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_link"/>
  <origin xyz="0.20 0.0 0.15" rpy="0 0 0"/>
</joint>
```

The parent and child names must match the scan message and Nav2 configuration.

## Static transform: useful test, bad permanent fix

You can temporarily test a sensor transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.0 --z 0.15 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id laser_link
```

Then verify:

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

The official Nav2 guide presents this as a quick demonstration, but recommends URDF and `robot_state_publisher` for actual robot projects because manually launching static publishers does not scale well. [docs.nav2](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/)

Use this test to answer:

> Is the missing TF edge the reason my costmap cannot process the scan?

If the costmap starts working after the test publisher begins, your URDF or `robot_state_publisher` setup is probably incomplete.

Do not leave both the test publisher and `robot_state_publisher` active for the same parent-child pair. Duplicate broadcasters can create warnings and unpredictable behavior.

## Frame-name configuration in Nav2

Check the relevant live parameters:

```bash
ros2 param get /global_costmap/global_costmap global_frame
ros2 param get /global_costmap/global_costmap robot_base_frame

ros2 param get /local_costmap/local_costmap global_frame
ros2 param get /local_costmap/local_costmap robot_base_frame

ros2 param get /amcl global_frame
ros2 param get /amcl odom_frame
ros2 param get /amcl base_frame_id
```

A typical configuration is:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
```

AMCL typically needs compatible names:

```yaml
amcl:
  ros__parameters:
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_link
```

The exact parameter names can vary by Nav2 distribution and node, so check the parameters actually exposed by your installed version rather than copying a configuration from another ROS 2 release.

## `base_link` versus `base_footprint`

Both are common, but inconsistency causes problems.

A common structure is:

```text
odom → base_footprint → base_link → laser_link
```

Here:

- `base_footprint` is the robot’s 2D ground projection.
- `base_link` represents the physical robot body.

Another setup may use:

```text
odom → base_link → laser_link
```

Both can work if every component agrees.

The failure occurs when:

- Gazebo publishes `odom → base_footprint`.
- AMCL expects `base_link`.
- Nav2 costmaps expect `base_footprint`.
- The sensor is attached to a different branch.

Choose the intended robot base frame and check:

```bash
ros2 param get /controller_server robot_base_frame
ros2 param get /global_costmap/global_costmap robot_base_frame
ros2 param get /local_costmap/local_costmap robot_base_frame
```

For a namespaced or custom robot, behavior-tree plugins can also contain a hardcoded base-frame name. A Nav2 issue documents a case where the configured robot base frame differed from the frame expected inside the behavior-tree configuration. [github](https://github.com/ros-navigation/navigation2/issues/4508)

## Error-to-fix guide

### `Invalid frame ID "map"`

Check:

```bash
ros2 run tf2_ros tf2_echo map odom
```

Likely fix:

- Start AMCL or SLAM Toolbox.
- Activate lifecycle nodes.
- Set the initial pose for AMCL.
- Confirm localization is configured to publish `map → odom`.

### `Invalid frame ID "base_link"`

Check:

```bash
ros2 run tf2_tools view_frames
```

Likely fix:

- Start `robot_state_publisher` or the odometry publisher.
- Change Nav2’s base frame to the actual frame.
- Correct the URDF link name.
- Remove unintended namespaces.

### `Could not transform laser scan`

Check:

```bash
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link <scan_frame>
```

Likely fix:

- Correct the scan’s frame.
- Add the sensor joint to URDF.
- Start `robot_state_publisher`.
- Fix the sensor topic or namespace.

### `Timed out waiting for transform`

Check:

```bash
ros2 run tf2_ros tf2_monitor map base_link
```

Likely fix:

- Repair the missing edge.
- Align simulation time.
- Check whether the broadcaster is publishing continuously.
- Confirm the requested frame names.

### `Extrapolation into the future`

Likely fix:

- Use consistent `/clock`.
- Set `use_sim_time` consistently.
- Avoid future-dated sensor or TF messages.
- Check system time and simulation startup order.

### `Extrapolation into the past`

Likely fix:

- Restart stale nodes after restarting Gazebo.
- Ensure all nodes use the same clock.
- Check delayed sensor data.
- Check transform publication frequency and timestamps.

A Nav2 issue shows both “frame does not exist” and “extrapolation into the past” in the same class of startup failure, which is why the exact error and timing state must be checked rather than treating all TF timeout messages identically. [github](https://github.com/ros-navigation/navigation2/issues/2560)

## A practical TF audit script

Run this after launching your simulation:

```bash
#!/usr/bin/env bash

echo "=== Required topics ==="
ros2 topic list | grep -E '^/(clock|scan|odom|tf|tf_static|map)$'

echo
echo "=== Topic rates ==="
ros2 topic hz /clock --window 5
ros2 topic hz /odom --window 5
ros2 topic hz /scan --window 5

echo
echo "=== Frame graph ==="
ros2 run tf2_tools view_frames

echo
echo "=== Required transforms ==="
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

Do not run all three `tf2_echo` commands in one terminal because each command continuously runs. Open separate terminals, or run them individually.

For your AMR, replace `laser_link` with the frame returned by:

```bash
ros2 topic echo /scan --once
```

## Recommended debugging order

Use this exact order:

```text
1. Is /clock advancing?
2. Is /odom being published?
3. Is /scan being published?
4. What is the scan frame_id?
5. Does odom → base_link exist?
6. Does base_link → scan_frame exist?
7. Is map → odom being published?
8. Are all required frames in one connected tree?
9. Are timestamps compatible?
10. Are Nav2 parameters using the same frame names?
```

The most important rule is:

> Debug TF edges, not just error messages.

For example, this error:

```text
Timed out waiting for transform from base_link to map
```

does not necessarily mean `map → base_link` itself is missing. It could mean any of these is broken:

```text
map → odom
odom → base_link
```

Similarly, a costmap error involving `base_link` may actually be caused by a missing `base_link → laser_link` relationship.

TF2 provides `tf2_echo` for numerical transform checks, `view_frames` for complete-tree inspection, and `tf2_monitor` for timing and broadcaster statistics. These are the primary tools you should use before changing Nav2 parameters. [docs.ros](https://docs.ros.org/en/rolling/p/tf2_ros/doc/cli_tools.html)