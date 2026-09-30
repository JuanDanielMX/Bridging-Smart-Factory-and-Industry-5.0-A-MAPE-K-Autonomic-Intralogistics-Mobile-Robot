# Autonomic TurtleBot3 Waffle Pi Lift Truck

This repository is a complete ROS Noetic implementation of the mobile intralogistics system described in **“Bridging Smart Factory and Industry 5.0: A MAPE-K Autonomic Intralogistics Mobile Robot.”** It targets the paper's TurtleBot3 Waffle Pi, two-Raspberry-Pi arrangement, host computer, lift mechanism, LiFePO4 power system, and Node-RED/MQTT supervision.

The attached paper was treated only as a technical source. Its four Self-CHOP behaviors are implemented as working software:

| Capability | Implementation |
|---|---|
| Self-Configuration | LiDAR SLAM/localization, a costmap-aware RRT `move_base` global-planner plugin, waypoint missions, and replanning after layout changes |
| Self-Healing | Encoder odometry versus LiDAR-corrected `map -> base_footprint` displacement, stuck classification, actuator stop, dashboard alert, and operator recovery acknowledgement |
| Self-Optimization | The paper's `Sf = 0.02 + 0.02B` m/s workload law, battery telemetry, and a final velocity limiter |
| Self-Protection | DWA/local-costmap avoidance, a LiDAR emergency velocity gate, MQTT ACLs, and AES-256-CBC payload protection strengthened with HMAC-SHA256 and replay checks |

The project also records MAPE-K events, battery data, alerts, orders, and speed decisions in SQLite.

## Repository Layout

```text
catkin_ws/src/autonomic_turtlebot3/
  include/, src/       RRT planner and reusable policies
  scripts/             MAPE-K, safety, host/aux MQTT, lift, battery, knowledge nodes
  config/              Navigation, planner, hardware, and communication values
  launch/              Simulation and three-computer deployment profiles
  missions/            Safe simulation mission and locked factory template
  test/                Policy and cryptography tests
docs/                   Architecture, hardware, deployment, and validation guides
mosquitto/              Broker listener and topic ACLs
node-red/               Importable supervisory dashboard
tools/                  Installation, build, secrets, broker, and validation scripts
```

## Fastest Test: Gazebo

Use Ubuntu 20.04 with ROS Noetic. The paper explicitly specifies Noetic; it is retained for reproducibility even though it is now an end-of-life ROS distribution.

```bash
./tools/install_noetic.sh host
./tools/build_workspace.sh
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch autonomic_turtlebot3 simulation.launch
```

After Gazebo, SLAM, and `move_base` are ready:

```bash
rosservice call /autonomic_manager/start_mission
```

The demo navigates an empty-world pallet loop and simulates the lift and battery. Set a workload at any time:

```bash
rostopic pub -1 /factory/queued_orders std_msgs/Int32 "data: 5"
```

Five queued orders produce the paper's fixed comparison speed of `0.12 m/s`; zero produces `0.02 m/s`, and eleven produces `0.24 m/s`.

## Real Robot

The physical system uses three roles:

| Computer | Main responsibility | Launch file |
|---|---|---|
| Raspberry Pi 1 on Waffle Pi | OpenCR, Dynamixel wheels, LiDAR, IMU, encoder odometry | `robot.launch` |
| Raspberry Pi 2 on lift truck | TB6600 lift, INA219 battery, encrypted MQTT link | `auxiliary.launch` |
| Ubuntu host computer | ROS master, SLAM/AMCL, RRT navigation, MAPE-K, MQTT, Node-RED, SQLite | `host_slam.launch` or `host_navigation.launch` |

Follow [Deployment](docs/DEPLOYMENT.md) for network setup, mapping, calibration, broker/dashboard setup, and launch order. Follow [Hardware](docs/HARDWARE.md) before connecting power or a stepper driver.

## Required Calibration

The paper does **not** provide mechanical drawings, fork dimensions, payload rating, motor/lead-screw specifications, exact battery sensor model, or real factory coordinates. This repository therefore keeps those values configurable and blocks the factory mission until calibration is explicitly confirmed.

Before a physical run:

1. Measure the finished robot and update the footprint in `config/costmap_common.yaml`.
2. Verify lift direction, both normally-closed limit switches, motor current, and the hardware emergency stop with wheels raised and no payload.
3. Calibrate `voltage_empty`, `voltage_full`, and battery capacity in `config/battery.yaml`.
4. Copy `missions/factory_template.yaml`, record poses from the finished map, and set `calibrated: true` only after checking every pose.
5. Run every acceptance test in [Validation](docs/VALIDATION.md) unloaded and at low speed before carrying a pallet.

The software will not make an uncharacterized mechanism safe. Use a fused supply, BMS, physical emergency stop, guards, mechanical end stops, and a load-rated structure.

## Documentation

- [Architecture and paper mapping](docs/ARCHITECTURE.md)
- [Hardware BOM and wiring](docs/HARDWARE.md)
- [Installation and deployment](docs/DEPLOYMENT.md)
- [Self-CHOP validation procedure](docs/VALIDATION.md)

## Local Verification

Static validation works without ROS:

```bash
python3 tools/validate_project.py
PYTHONPATH=catkin_ws/src/autonomic_turtlebot3/src \
  python3 -m unittest discover -s catkin_ws/src/autonomic_turtlebot3/test -v
```

On Ubuntu/ROS, `tools/build_workspace.sh` additionally compiles the C++ planner and resolves package dependencies.

Licensed under the [MIT License](LICENSE).
