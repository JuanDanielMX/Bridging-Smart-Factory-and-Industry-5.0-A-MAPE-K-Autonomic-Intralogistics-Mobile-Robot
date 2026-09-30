# Paper-Matched Validation

Run these tests in simulation first, then on the physical platform unloaded, with a spotter and accessible hardware emergency stop. Record a rosbag, Node-RED screen capture, and SQLite database for each trial.

## Preflight

```bash
rostopic hz /scan /odom /battery_state
rosrun tf tf_echo map base_footprint
rosparam get /move_base/base_global_planner
rostopic echo /autonomic/state
```

Pass only if sensor rates are stable, TF is continuous, the planner is `autonomic_turtlebot3/RRTGlobalPlanner`, both lift limits stop motion, and a mission cancel produces zero `/cmd_vel`.

## 1. Self-Configuration

Objective: show that the robot maps the workspace, computes an RRT route inside navigation constraints, and completes the pallet loop after a layout change.

1. Start mapped navigation or online SLAM and one calibrated mission.
2. Display the global costmap and `RRTGlobalPlanner/plan` in RViz.
3. Place a static obstacle so it changes the available route but leaves a safe alternative.
4. Clear the costmap if needed: `rosservice call /move_base/clear_costmaps`.
5. Repeat the mission.

Pass criteria:

- LiDAR marks the obstacle and the robot never crosses lethal/inflated cells.
- A new RRT plan is generated.
- The robot reaches each mission pose without contact or restricted-zone violation.
- MAPE-K events contain `rrt_route_requested` and `route_completed`.

## 2. Self-Healing

Objective: reproduce wheel-motion versus effective-motion divergence and human-assisted recovery.

1. Lower speed and use an unloaded platform.
2. Introduce a low obstruction that prevents translation without blocking the LiDAR scan plane. Do not restrain a robot at high torque.
3. Confirm encoder odometry changes while map-frame displacement remains nearly fixed.
4. Wait for state `STUCK` and the dashboard alert.
5. Remove the obstruction by hand, inspect the robot, then press **Recovery complete** or call the acknowledgement service.

Pass criteria:

- Detection occurs after the configured window, not during ordinary stops.
- `move_base` is cancelled and `/cmd_vel` becomes zero.
- The dashboard requests operator assistance.
- The mission does not resume before acknowledgement and retries the interrupted action afterward.
- The knowledge database stores `stuck_detected`, `operator_alert`, and `operator_recovery_acknowledged`.

Tune thresholds from bagged `/odom`, `/tf`, and `/cmd_vel` if false positives occur.

## 3. Self-Optimization

Objective: verify the workload speed law and compare energy-supported operating time.

Expected values:

| Queued orders `B` | Target speed m/s |
|---:|---:|
| 0 | 0.02 |
| 1 | 0.04 |
| 5 | 0.12 |
| 10 | 0.22 |
| 11 | 0.24 |

Publish each order count and inspect `/autonomic/target_speed`. For an energy experiment matching the paper, keep map, route, payload, initial battery state, order sequence, floor, and obstacle conditions constant. Compare repeated runs at fixed `0.12 m/s` with repeated runs using modulation, ending each run at the same battery threshold.

Pass criteria:

- Every target follows `0.02 + 0.02B` and remains in `0.02-0.24 m/s`.
- The final `/cmd_vel` never exceeds the target.
- Battery, queued orders, and speed decisions are recorded.
- The modulated strategy shows a measured improvement under the local test conditions. Do not claim the paper's `12.4%` result unless this hardware's experiment independently produces it.

At 15% battery or lower, this implementation adds a documented safety/energy cap of `0.06 m/s`.

## 4. Self-Protection

### Physical

1. Command a low-speed route.
2. Place an obstacle in the direction of travel outside the emergency distance.
3. Observe DWA/RRT adaptation.
4. If the obstacle enters the configured emergency distance, verify `/autonomic/emergency_stop` becomes true and `/cmd_vel` is zero.

Pass only if there is no collision, the costmap reflects the obstacle, route/motion changes safely, and removal clears the velocity gate.

### Digital

1. Subscribe to `factory/secure/#` and `robot/secure/#` using authorized broker credentials.
2. Confirm packets expose only envelope fields (`alg`, `ct`, `iv`, `nonce`, `tag`, `ts`, `v`) rather than operational values.
3. Alter one ciphertext character and publish it; the bridge must reject it.
4. Republish a previously accepted control envelope; replay protection must reject it.
5. Attempt cross-direction publish/read with each MQTT account; ACLs must deny it.

Pass only if valid values decrypt correctly and tampered, stale, replayed, and unauthorized messages are rejected.

## Evidence Queries

```bash
sqlite3 ~/.ros/autonomic_turtlebot3/knowledge.db \
  "SELECT datetime(ros_time,'unixepoch'), category, payload FROM events ORDER BY id DESC LIMIT 50;"
```

Count events by category:

```bash
sqlite3 ~/.ros/autonomic_turtlebot3/knowledge.db \
  "SELECT category, COUNT(*) FROM events GROUP BY category ORDER BY category;"
```

Archive the map, mission YAML, all config files, bag files, battery calibration, payload mass, software commit/version, and database together. Without those conditions, repeated energy and navigation results are not comparable.
