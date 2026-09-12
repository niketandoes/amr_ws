# Prompt: 
I want you to debug the whole project, focus on nav2 stack more, Check the related repos attached below for ref only, 
do not perform any type of automated work without permission from me, if you want to access terminal, ask first, you work is to list all the bugs, potentail failures, suspicious code which may create problems in future. 

# Expected Outcome of the project 
Here's a compact, scannable checklist — organized by file/phase so you can paste this straight into Cursor and have it check each block against the actual code.

## Expected Outcomes Checklist

**`intent_broadcaster.py`**
- [ ] Publishes to `/fleet/intent` at 5Hz, only after TF (`map → {robot}/base_footprint`) resolves
- [ ] `current_state` = 0 (normal) / 1 (approaching) / 2 (in-choke) — never hardcoded
- [ ] `is_in_choke_zone` boolean matches the same bounding box used in the coordinator
- [ ] `approach_request_time` = 0.0 outside approach zone; holds one fixed monotonic value while inside it (doesn't reset every tick)
- [ ] `plan_callback` never IndexErrors on an empty/short `msg.poses`

**`decentralized_coordinator.py`**
- [ ] No two robots ever report `is_in_choke_zone=True` simultaneously (zero overlap — check across logs/CSV)
- [ ] On cold start, a robot with an incomplete peer picture holds (yields to `UNKNOWN_PEER`) instead of proceeding — bounded by `APPROACH_PEER_GRACE_SEC`
- [ ] Exit requires `HYSTERESIS_TICKS_REQUIRED` consecutive clear ticks before resuming — not a single reading
- [ ] Tie-break resolves by earliest `approach_request_time`; identical timestamps fall back to string ID
- [ ] Stale peers (>`INTENT_EXPIRY_SEC`) get pruned and don't cause permanent lockup
- [ ] `cmd_vel_coord` halts are only published while actually yielding — goes silent otherwise (so `twist_mux` timeout can hand back control)

**`twist_mux_config.yaml` / launch**
- [ ] `controller_server` remapped to `cmd_vel_nav`, coordinator to `cmd_vel_coord`
- [ ] Priorities: coordinator (20) > navigation (10)
- [ ] Both topics have a `timeout` set — a stale/dead publisher can't hold the lock forever

**`task_allocator_cnp.py`**
- [ ] Blockage detected only when velocity < 0.05 m/s for > 4.0s **with an active goal** (not just idle)
- [ ] Auction window closes and awards lowest `bid_cost` within its configured wait time
- [ ] Winning peer actually dispatches a new Nav2 goal on award — verify no silent no-op
- [ ] Blocked robot re-routes to a standby pose, doesn't just idle in the corridor

**`battery_simulator.py`**
- [ ] Battery % strictly decreases with motion, never goes negative, never exceeds 100
- [ ] Drain rate scales with actual velocity/activity, not a flat timer

**`fleet_dashboard_server.py`**
- [ ] SSE stream pushes at 10Hz without client polling
- [ ] Position, battery, and status shown match the underlying ROS2 topic values exactly (spot-check one robot manually)

**`benchmark_logger.py` / `benchmark_results.csv`**
- [ ] `Min_Inter_Robot_Dist_m` actually varies row to row — flag immediately if it's constant 0 again
- [ ] Every row has a real `Outcome`, `Task_Completion_Time_s`, `Distance_Traveled_m` — no blank/null fields
- [ ] Across ≥10 repeated trials: 0 `ABORTED` due to collision, and average completion time ≥20% below the stop-and-wait baseline

**Cross-cutting**
- [ ] `colcon build --symlink-install` run after any `.msg` change — stale interface bindings are a common silent-failure source
- [ ] No node throws on startup if a peer node hasn't launched yet (should log a warning, not crash)

*"Check each unchecked item against the actual code and tell me which ones fail, with file + line number — don't just confirm the ones that already look fine."* 

# Relevent repos 

rbot – “An open-source Autonomous Mobile Robot simulation stack for ROS 2 Jazzy and Gazebo Harmonic.”
https://github.com/rlxai/rbot

autonomous-warehouse-amr – Differential-drive AMR with LiDAR, Nav2, SLAM, and a warehouse world (ROS 2 Jazzy + Gazebo Harmonic).
https://github.com/Anastasios03git/autonomous-warehouse-amr

ros2_amr_mecanumbot – Mecanum-wheel AMR simulation (and hardware) with SLAM, Nav2, QR-based goals, and a lift, in ROS 2 Humble + Gazebo.
https://github.com/Trkkhrmn/ros2_amr_mecanumbot

amr_robot – Differential-drive AMR for warehouse logistics simulation using ROS 2, Gazebo Harmonic, and Nav2 (includes docking and mission orchestration).
https://github.com/mohamedazimal27/amr_robot

amr (GradVizor) – Self-driving ground robot implementation using ROS 2 and Gazebo for autonomous navigation tasks.
https://github.com/GradVizor/amr

Also relevant (AMR platforms / stacks with simulation)
OpenAMRobot (software) – Open-source mobile robotics platform on ROS 2 Jazzy with Gazebo simulation, Nav2, SLAM, docking, and a web UI.
Software repo: https://github.com/openAMRobot/openamr-platform-sw

multi-robot-fleet-ros2 – Monorepo with AMR + mobile manipulator fleet simulation (Nav2, SLAM Toolbox, Open-RMF, Gazebo Harmonic worlds).
https://github.com/darshmenon/multi-robot-fleet-ros2

ros2-essentials – ROS 2 Humble essentials for AMRs and manipulators, with simulation-to-reality reuse (Isaac Sim / Isaac Lab oriented).
https://github.com/j3soon/ros2-essentials

donar_description – Differential-drive mobile robot simulation package for ROS 2 Humble + Gazebo Ignition (good baseline for Nav2/SLAM experiments).
https://github.com/Fonyuy45/donar_description
