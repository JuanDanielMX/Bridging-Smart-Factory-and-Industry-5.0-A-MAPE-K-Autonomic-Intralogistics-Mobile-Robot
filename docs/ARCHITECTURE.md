# Architecture and Paper Mapping

## System Flow

```text
Smart Factory / Node-RED
  encrypted orders + operator commands
              |
          Mosquitto
              |
        mqtt_bridge.py
              |
  +-----------+----------------------------+
  |           MAPE-K host                  |
  |                                        |
  | Monitor  LiDAR, odom, TF, battery,     |
  |          queued orders                 |
  | Analyze  obstacle, stuck divergence,   |
  |          demand, energy state          |
  | Plan     RRT path, assisted recovery,  |
  |          target speed, secure payload  |
  | Execute  move_base, safety filter,     |
  |          lift command, MQTT publish    |
  | Knowledge SQLite events + map + mission|
  +-----+----------------------+-----------+
        | TCPROS              | encrypted MQTT
  Raspberry Pi 1         Raspberry Pi 2
  OpenCR / LiDAR         MQTT bridge -> lift / battery
```

## ROS Nodes

| Node | Inputs | Outputs | Responsibility |
|---|---|---|---|
| `move_base` + `RRTGlobalPlanner` | map, scan, TF, goals | `/cmd_vel_raw`, global plan | Feasible RRT path and DWA local avoidance |
| `autonomic_manager.py` | odom, TF, battery, orders, lift state | goals, target speed, lift commands, alerts | MAPE-K mission state machine and Self-CHOP policy coordination |
| `safety_filter.py` | `/cmd_vel_raw`, scan, target speed, autonomy state | `/cmd_vel`, emergency state | Last-stage collision stop and speed enforcement |
| `mqtt_bridge.py` | protected MQTT and ROS state | orders/control into ROS; protected status/events to MQTT | Factory integration and application-layer security |
| `auxiliary_mqtt_bridge.py` | protected lift command topic; local lift/battery topics | local lift commands; protected auxiliary status | Paper-matched host-to-Raspberry-Pi-2 MQTT transport |
| `lift_controller.py` | lift commands, limit switches | step/direction signals and lift state | Bounded TB6600 actuation with timeout and hard limits |
| `battery_monitor.py` | INA219 | `/battery_state` | LiFePO4 voltage/current and estimated charge |
| `knowledge_logger.py` | state, events, orders, speed, battery | SQLite database | MAPE-K knowledge history |

`move_base` is deliberately remapped to `/cmd_vel_raw`. Only `safety_filter.py` publishes the final `/cmd_vel`, so no planner command can bypass the speed and LiDAR checks.

## Self-CHOP Behavior

### Self-Configuration

The global plugin samples free global-costmap cells, extends an RRT with collision-checked segments, joins the goal within tolerance, smooths line-of-sight segments, and returns a standard `nav_msgs/Path`. New LiDAR obstacles update costmaps; `move_base` invokes the planner again when a route becomes invalid.

Mission poses and lift actions are external YAML knowledge. The real template has `calibrated: false`; the manager refuses to start it until an operator replaces all coordinates and marks it calibrated.

### Self-Healing

The manager compares motion over a configurable three-second window:

- Encoder-derived `/odom` displacement must be at least `0.08 m`.
- LiDAR-corrected map-frame displacement must remain at or below `0.025 m`.
- Commanded motion must be at least `0.03 m/s`.

When all conditions hold, the manager cancels navigation, commands the lift to stop, sets state `STUCK`, publishes an operator alert, and waits. Recovery resumes only through the acknowledgement service/dashboard after a person clears the obstruction.

The thresholds are starting points, not universal constants. Tune them from recorded data after the final payload and floor surface are known.

### Self-Optimization

The target speed directly reproduces the paper:

```text
Sf = 0.02 + 0.02B m/s
```

`B` is bounded to 0-11. At or below 15% battery, the implementation additionally caps speed at `0.06 m/s`. The safety filter clamps planner output to that target without reconfiguring the local planner during a run.

### Self-Protection

Physical protection is layered:

- Inflated global/local costmaps represent the fork-extended footprint.
- DWA reacts to local obstacles.
- The final velocity filter stops inside `0.22 m` and scales speed between `0.22-0.55 m` in the direction of travel.
- Loss of velocity commands and any non-running autonomy state produce zero velocity.

Digital protection retains AES-256-CBC for paper compatibility and adds the integrity mechanism that CBC lacks. A 256-bit master key is expanded with HKDF into separate encryption and HMAC keys. Every envelope uses a random IV and nonce, HMAC-SHA256, a timestamp window, and a replay cache. Mosquitto ACLs separate robot and dashboard permissions. TLS can be enabled for transport protection outside an isolated laboratory LAN.

## State Machine

```text
IDLE --start--> RUNNING --all actions--> COMPLETE
                    |
                    +--motion divergence--> STUCK
                    |                         |
                    |          operator clears + acknowledges
                    |                         |
                    +<------------------------+
                    |
                    +--navigation/lift timeout--> ERROR

RUNNING/STUCK/ERROR --cancel--> IDLE
```

## Topics and Services

| Interface | Type | Purpose |
|---|---|---|
| `/factory/queued_orders` | `std_msgs/Int32` | Workload demand `B` |
| `/factory/assembly_ready` | `std_msgs/Bool` | Releases the mission after the assembly operation |
| `/autonomic/target_speed` | `std_msgs/Float32` | Calculated speed law result |
| `/autonomic/state` | `std_msgs/String` | State machine status |
| `/autonomic/alert` | `std_msgs/String` | Human-facing fault message |
| `/autonomic/mapek_event` | JSON in `std_msgs/String` | Structured MAPE-K audit event |
| `/autonomic/emergency_stop` | `std_msgs/Bool` | LiDAR velocity-gate status |
| `/lift/command` / `/lift/state` | `std_msgs/String` | Lift request and feedback |
| `/autonomic_manager/start_mission` | `std_srvs/Trigger` | Start one pallet loop |
| `/autonomic_manager/cancel_mission` | `std_srvs/Trigger` | Cancel and stop |
| `/autonomic_manager/acknowledge_recovery` | `std_srvs/Trigger` | Human-in-the-loop recovery |

## Deliberate Extensions Beyond the Paper

The source paper describes the concepts but omits several controls required for a responsible build. This implementation adds normally-closed lift limit switches, lift travel timeout, master-key derivation, message authentication, replay rejection, MQTT ACLs, a final velocity gate, locked uncalibrated missions, low-battery derating, and persistent event logging. These additions preserve the study behavior while making failures observable and bounded.
