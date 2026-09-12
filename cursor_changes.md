# Plan.md audit notes (static code review)

Source: checklist in `Plan.md`. Method: read the actual sources and `benchmark_results.csv`. **No simulation, no `colcon`, no reference-repo clones.** Runtime items are judged from code plus existing CSV, not from a live run.

Debug instrumentation that was briefly added to `intent_broadcaster.py` / `decentralized_coordinator.py` was **reverted**. This file is the only intended artifact.

---

## Checklist items that fail

### `decentralized_coordinator.py` — “no two robots `is_in_choke_zone=True` at once”

**Fails.** Occupancy is a local heuristic on BEST_EFFORT DDS, not a lock.

- Rule 2 still runs when **this robot is already in the choke**. Approach includes the choke box (`abs(x) ≤ 3.5` and `abs(y) ≤ 1.5`), so an occupant can lose the timestamp sort and **halt inside the corridor** while a peer is still gated by Rule 1. Classic deadlock / overlap.

```215:233:src/amr_simulation/scripts/decentralized_coordinator.py
        # ── RULE 2: Fair tie-break — earliest requester wins ──
        ...
        if self.robot_id != winner_id:
            self.yielding_to = winner_id
            self._halt(winner_id)
```

- Hysteresis treats **every peer with `< 5` clear ticks as “clearing”**, including peers that were **never** in the zone. First ~500 ms after a peer appears (or after prune+rejoin) every approaching robot yields as if the corridor is occupied.

```154:213:src/amr_simulation/scripts/decentralized_coordinator.py
            else:
                self.peer_clear_ticks[peer_id] = self.peer_clear_ticks.get(peer_id, 0) + 1
        ...
            peer_clearing = not peer_in_zone and not self._peer_confirmed_clear(peer_id)
            if peer_in_zone or peer_clearing:
                if not i_am_in_zone:
                    self.yielding_to = peer_id
                    self._halt(peer_id)
```

- Choke AABB `y ∈ [-1.2, 1.2]` is **larger than the real gap**. Walls leave about **y ∈ [-0.66, 0.66]** (`warehouse.sdf` ~195–217). Two robots can both be “in choke” on opposite sides of the wall, or inside wall volume, without sharing the opening.

Physical gap from SDF:

- `choke_wall_top` pose y=3.36, size y=5.4 → inner edge y ≈ +0.66
- `choke_wall_bottom` pose y=-3.36, size y=5.4 → inner edge y ≈ -0.66
- Gap width ≈ **1.32 m** (comments say 1.15 m)
- Walls occupy x ∈ [-0.2, 0.2]

Map generator (`scripts/generate_map.py` 37–39) uses the same ±0.66 / ±0.2 box.

---

### `decentralized_coordinator.py` — exit hysteresis

**Partial fail.** `HYSTERESIS_TICKS_REQUIRED = 5` exists, but it is not “N consecutive clear ticks **after leaving the zone**”. It is a global counter from first sighting. That will false-yield on cold start and after `INTENT_EXPIRY_SEC` prune (lines 132–146).

---

### `intent_broadcaster.py` / coordinator — `current_state` / approach clock

**State 0/1/2 is computed, not hardcoded** (`intent_broadcaster.py` 141–147) — that checklist box **passes**.

Related failures the checklist did not name:

- `FleetIntent.msg` defines `3 = YIELDING` and `4 = BLOCKED`; the broadcaster never sets them. Dashboard maps them (`fleet_dashboard_server.py` 167) so UI status will stay `NORMAL_NAV` / `APPROACHING_CHOKE` / `IN_CHOKE` while the coordinator is actually halted.
- `approach_request_time` uses `time.monotonic()` (`intent_broadcaster.py` 125), not `/clock`. Fine on one machine; wrong if sim is paused (grace still elapses) or if processes are split across hosts.
- Pose comes from TF `map → {robot}/base_footprint`; velocity is copied from `odom` (odom frame). Mixed frames for peers.

---

### `twist_mux_config.yaml` / launch — remaps

**Multi-robot path passes** (`nav2_bringup.launch.py` 102–103, `single_amr.launch.py` 160–167, yaml priorities 20/10, timeouts 0.5 s).

**Fails for the other bringup:** `warehouse_world.launch.py` bridges Gazebo **`/cmd_vel`** (line 59) but `nav2_bringup` remaps the controller to **`cmd_vel_nav`** and this launch **never starts `twist_mux`**. Single-robot warehouse launch: Nav2 output never reaches the robot.

**Nav2 recovery bypass (multi-robot too):** only `controller_server` is remapped to `cmd_vel_nav`. `behavior_server` is not (`nav2_bringup.launch.py` 126–133). BT `Spin` / `BackUp` (`navigate_to_pose.xml` 25–28) publish on namespaced **`cmd_vel`**, which is the **mux output** to Gazebo — recoveries fight or skip the coordinator.

After a yield ends, mux waits `timeout: 0.5` with no `cmd_vel_coord` before handing back to nav — a half-second of “dead” cmd after resume.

---

### `task_allocator_cnp.py` — blockage only with an active goal

**Fails for the real mission path.** Stall logic is `is_navigating and active_goal_pose` (142–156), but those flags are set only in `goal_pose_callback` (130–135). The dashboard opposing test and CNP dispatch use **`NavigateToPose` actions**, not `goal_pose`. Those runs never arm the 4 s stall detector unless something also publishes `goal_pose`.

Yielding at the choke (`speed ≈ 0` for `> 4 s`) will also look like a blockage **if** `goal_pose` was used — auctions during a legal halt.

Thresholds in code: velocity `< 0.05 m/s` for `> 4.0 s` (matches checklist numbers) **when the flags are set**.

---

### `task_allocator_cnp.py` — winner actually gets the auctioned goal

**Fails (silent wrong goal).**

- `TaskAward.msg` has only `task_id`, `winner_id`, `stamp` — **no `target_pose`**.
- Winner always goes to **`(3.5, -1.5)`**, not `msg.target_pose` from the auction (334–349).
- `dispatch_nav2_goal` is `send_goal_async` with **no accept/result callback** (351–359). Server down or rejected goal = warning or nothing; mission continues as if awarded.
- Auctioneer **does not cancel** the stuck Nav2 goal before standby.

---

### `task_allocator_cnp.py` — blocked robot goes to standby

**Fails on the 0-bid path.** `close_auction` logs “Retrying in 2.0s” then sets `active_auction = None` and **returns with no timer** (275–278). `is_blocked` stays True, `is_navigating` False, **`reroute_to_standby` never runs**. Robot sits in the corridor.

Standby after a **successful** award is implemented (282–304):

- amr1 → (-3.5, 2.0)
- amr2 → (3.5, 2.0)
- else → (0.0, -3.5)

Auction window when bids exist: 0.5 s timer, sort by `bid_cost`, publish award — **passes** (185, 251–274).

---

### `fleet_dashboard_server.py` — telemetry matches ROS exactly

**Fails “exactly”.** TF pose is `round(..., 2)` / yaw `round(..., 3)` (138–147). Status cannot show yielding/blocked (see above). Path is hardcoded to `/home/niket/amr_ws` (348–352).

SSE at 10 Hz without polling **passes** (273–287, `time.sleep(0.1)`).

Other dashboard issues:

- `/amr1/trigger_blockage` is hardcoded (99, 244–248); blockage button only hits AMR 1.
- `discover_robots` can add duplicate subscriptions if a robot is purged and rediscovered.
- `event_queue` grows without bound if no SSE client is connected.
- `dispatch_opposing_test` also uses action client, so CNP stall flags are not set.

---

### `benchmark_logger.py` / `benchmark_results.csv`

**Fails, with file evidence.**

Existing CSV: **every** `Min_Inter_Robot_Dist_m` is **`0.00`**. 18 rows, **17 `ABORTED`**, 1 `SUCCEEDED`. Duplicate identical rows (e.g. `amr2,ABORTED,26.31` three times).

That fails:

- min distance must vary (not stuck at 0)
- ≥10 trials with **0 collision ABORTED**
- average TCT **≥20% under** stop-and-wait (no baseline in repo, and almost all trials aborted)

Logger issues that explain garbage metrics:

- Min distance only while **both** have `active_goal` (79–89). One robot finishing → freeze or `N/A`; overlapping at a cell → `0.00`.
- CSV path is cwd-relative `'benchmark_results.csv'` (138).
- `v = dist / 0.1` assumes the TF timer always fires at 10 Hz (72–73).
- `benchmark_logger` is **installed** (`CMakeLists.txt`) but **not** started from `single_amr.launch.py` / `multi_robot_warehouse.launch.py` — easy to run extra copies and duplicate rows.
- Outcome / TCT / distance are written when a goal ends; fields are not blank in the current CSV (that sub-item is OK as formatting). `N/A` is used if min dist stays `inf`.

---

### Cross-cutting / Nav2

| Issue | Where | Why it matters |
|---|---|---|
| No AMCL; fake localization | `nav2_bringup.launch.py` 58–74 static `map → {ns}/odom`; AMCL in yaml never launched | Drift/slip → plan in the wrong place; CSV aborts |
| Choke barely fits inflated footprint | footprint ~0.6×0.5 m, `inflation_radius: 0.35` (`nav2_params.yaml` 102, 121, 155); gap ~1.32 m | Half-width 0.25 + inflation 0.35 = 0.60 vs half-gap 0.66 → ~6 cm/side; RPP `use_collision_detection: true` (line 80) → **ABORTED** (matches CSV) |
| Halt vs progress checker | coordinator halt + `movement_time_allowance: 60` (`nav2_params.yaml` 59–61) | Long yield → Nav2 **ABORTED** while “coordinating” |
| `transform_tolerance: 0.1` | controller / behavior (75, 189) | Sim TF jitter → missing transforms |
| BT XML is BT.CPP 3 style | `navigate_to_pose.xml` line 4 `main_tree_to_execute` | Jazzy Nav2 expects BT.CPP 4 (`BTCPP_format="4"`) — tree may fail to load |
| Namespaced params dump | `single_amr.launch.py` 97–107 wraps yaml under `{namespace: ...}` | If ROS 2 already namespaces, plugins silently get **unprefixed** `base_footprint` / `/scan` |
| `fleet_robot_ids` never passed | coordinator default `['amr1','amr2','amr3']` (73–75); launch only sets `use_sim_time` | Disable `amr3` → 2 s `UNKNOWN_PEER` then “proceed incomplete” |
| `warehouse_world.launch.py` vs mux | as above | Single-robot Nav2 dead |
| `velocity_smoother` in yaml, not launched | `nav2_params.yaml` / `nav2_bringup.launch.py` | Unused; not a crash |
| `update_params_dict` | `single_amr.launch.py` 11–36 | Rewrites `odom`/`scan`/`base_frame` under namespace; `global_frame: map` stays shared `map` (correct if static TFs are right) |

Nodes generally **do not crash** if a peer is missing (warnings / empty intents). That cross-cut item is OK.

`colcon build --symlink-install` after `.msg` changes is a process note, not something the code enforces.

---

## Checklist items that hold in code (not runtime-proven)

- **5 Hz after `map → {robot}/base_footprint`:** `intent_broadcaster.py` 77–82, 106 (`create_timer(0.2, ...)` only after TF).
- **Choke constants match** between broadcaster 33–36 and coordinator 48–51 (duplicated, so they can drift later). Approach limits 3.5 / 1.5 also match.
- **`plan_callback` empty path:** slice `msg.poses[step:step*4:step]` (95–97) does not `IndexError`. Short paths can yield **zero** waypoints (silent empty intent).
- **`approach_request_time` sticky while in the approach rectangle:** 122–127, 150. Reset to 0.0 when `not in_approach`. Because approach **includes** choke, the timestamp is held while occupying the corridor too.
- **Tie-break sort `(timestamp, id)`:** coordinator 226–228. String ID breaks exact ties.
- **Cold start `UNKNOWN_PEER`:** coordinator 183–195, bounded by `APPROACH_PEER_GRACE_SEC = 2.0`.
- **Stale intent prune 1.5 s:** 57, 132–146. After prune the code **assumes clear** — dangerous if packets dropped while a peer is still in the gap.
- **`cmd_vel_coord` only from `_halt`:** 235–240; silent when not yielding (mux timeout can release). If `yielding_to` is still set, halt continues even after leaving the approach box (yielding check runs first).
- **Mux priorities + timeouts (fleet launch):** `twist_mux_config.yaml` 1–12; `use_stamped: true`; coordinator publishes `TwistStamped`.
- **Auction awards lowest `bid_cost` after 0.5 s** when bids exist: 185, 251–274.
- **Battery:** only decreases, `max(0.0, …)` (68–78); drain scaled by linear/angular speed; never programmed above 100 (no charger). Idle still drains (`drain_rate_idle`). `BatteryState.percentage` is 0.0–1.0 (correct ROS convention). amr2/amr3 default SOC 86% / 92% if `initial_percentage` stays 100.
- **SSE 10 Hz:** dashboard 273–287.
- **Controller remap to `cmd_vel_nav`:** `nav2_bringup.launch.py` 102–103.

---

## Highest-risk Nav2 / fleet failures (priority)

1. **Physical choke vs costmap inflation** + RPP collision abort → CSV sea of `ABORTED`.
2. **Coordinator Rule 2 + false “clearing” hysteresis** → overlap, deadlock, or halt in the gap.
3. **CNP award ignores the auction pose; 0-bid leaves the robot in the lane.**
4. **`behavior_server` / single-robot launch skip `twist_mux`.**
5. **`Min_Inter_Robot_Dist_m` still glued at `0.00` in `benchmark_results.csv`.**

---

## File-by-file suspicious / future-breakage notes

### `intent_broadcaster.py`

- Choke/approach boxes copied as class constants (sync hazard with coordinator).
- `current_state` never 3/4 despite message docs.
- Downsample `poses[step:step*4:step][:3]` is not “t+1s, t+2s, t+3s” as README claims; it is index striding.

### `decentralized_coordinator.py`

- README still says “lower string ID wins” and “50 Hz”; code is earliest `approach_request_time` at **10 Hz**.
- `fleet_robot_ids` default not wired from launch.
- Rule 1 occupancy + Rule 2 tie-break both apply while occupant is in approach.

### `task_allocator_cnp.py`

- Battery in bid: `msg.percentage * 100.0` — correct for this simulator; would explode if a driver published 0–100 already.
- Low-battery dock poses hardcoded; `is_blocked = True` prevents new auctions.
- `create_timer(0.5, close_auction)` is periodic until cancelled; 0-bid path cancels and never retries despite the log line.

### `battery_simulator.py`

- No recharge path; once at 0% it stays 0 with `POWER_SUPPLY_STATUS_DISCHARGING`.
- 1 Hz wall timer vs `use_sim_time` stamps.

### `fleet_dashboard_server.py`

- Hardcoded workspace path; port fallback 8080/8081/8082.
- Position from TF (map), speed from odom — can disagree if TF stale.

### `benchmark_logger.py`

- Hardcoded `['amr1','amr2','amr3']`.
- Deadlock accounting adds 0.5 s per 0.5 s tick after 3 s standstill (can over-count).
- Status uses `status_list[-1]` only.

### `nav2_params.yaml`

- `min_y_velocity_threshold: 0.5` (typical diff-drive ignore-y).
- Local costmap `global_frame: odom` rewritten to `{ns}/odom`.
- Global costmap `update_frequency: 1.0` — slow to see other robots as obstacles.
- Obstacle `raytrace_max_range: 3.0` / `obstacle_max_range: 2.5`.
- NavFn `allow_unknown: true` + `track_unknown_space: true`.

### `navigate_to_pose.xml`

- Recovery: clear costmaps, Spin 1.57 rad, Wait 5 s, BackUp 0.30 m at 0.05 m/s.
- `RecoveryNode number_of_retries="6"` — long stuck time before abort.

### `amr.xacro`

- Diff drive topic `/$(arg robot_name)cmd_vel` with launch `robot_name:={name}/` → `/amr1/cmd_vel` (slash from the arg). Fragile if someone passes `amr1` without trailing slash.

### `warehouse.sdf`

- Comment “1.15 m choke”; geometry is ~1.32 m. Coordinator AABB is larger still.

### Launch / install

- Scripts installed as PROGRAMS in `CMakeLists.txt` including `benchmark_logger.py`, `coordinated_choke_test.py`, `benchmark_runner.py`.
- Fleet launch: amr1 (-4, 0), amr2 (4, 0, yaw π), amr3 (1, -4, yaw π/2).

---

## Existing `benchmark_results.csv` snapshot (evidence)

Columns: `Robot_ID, Outcome, Task_Completion_Time_s, Deadlock_Duration_s, Distance_Traveled_m, Min_Inter_Robot_Dist_m`

- All `Min_Inter_Robot_Dist_m` = `0.00`
- Outcomes almost entirely `ABORTED` (one `SUCCEEDED` for amr2, TCT 11.57 s, min dist still 0.00)
- Duplicate rows with identical TCT/distance (multiple logger instances or repeated export)

---

## Scope / what was not done

- No `colcon build`, no `ros2 launch`, no terminal experiments (per `Plan.md`).
- Reference GitHub AMR repos were not cloned; used only as named context.
- Item “1” (choke vs inflation / abort cause) was selected for a future runtime pass; **no code was left instrumented**.
