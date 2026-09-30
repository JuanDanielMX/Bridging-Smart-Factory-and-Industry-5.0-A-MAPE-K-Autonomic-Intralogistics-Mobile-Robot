# Installation and Deployment

## Operating-System Baseline

The paper specifies ROS Noetic. Use Ubuntu 20.04 on the host and both Raspberry Pis for the closest reproduction. Because Noetic and Ubuntu 20.04 are legacy platforms, keep this system on an isolated factory/laboratory VLAN, apply available security updates, and use the TLS listener when traffic crosses an untrusted network.

Run the role installer on each machine from the project root:

```bash
# Ubuntu host computer
./tools/install_noetic.sh host

# TurtleBot3 Raspberry Pi 1
./tools/install_noetic.sh robot

# Lift/battery Raspberry Pi 2
./tools/install_noetic.sh auxiliary
```

Build the workspace on every machine:

```bash
./tools/build_workspace.sh
source catkin_ws/devel/setup.bash
```

Install matching OpenCR firmware using the normal TurtleBot3 Waffle Pi Noetic procedure before launching the base.

## ROS Network

Give all machines fixed IP addresses or DHCP reservations. The example below uses:

- Host: `192.168.10.10`
- Raspberry Pi 1: `192.168.10.21`
- Raspberry Pi 2: `192.168.10.22`

On every machine, set the host as ROS master. Set `ROS_IP` to that machine's own address:

```bash
export ROS_MASTER_URI=http://192.168.10.10:11311
export ROS_IP=192.168.10.21  # change per machine
export TURTLEBOT3_MODEL=waffle_pi
source /opt/ros/noetic/setup.bash
source /path/to/Paper_1/catkin_ws/devel/setup.bash
```

Verify forward and reverse hostname/IP reachability and synchronize clocks with NTP. The encrypted MQTT replay window is 120 seconds by default.

## Secrets and Broker

On the host:

```bash
./tools/generate_runtime_env.sh
./tools/setup_broker.sh
set -a
source "${XDG_CONFIG_HOME:-$HOME/.config}/autonomic_turtlebot3/runtime.env"
set +a
```

The generated file is mode `0600`. Back it up through a secrets manager, not source control. The same `AUTONOMIC_MASTER_KEY_HEX` must be present in the environment that starts ROS and Node-RED.

Create a separate mode-`0600` environment file on Raspberry Pi 2 containing only `AUTONOMIC_MASTER_KEY_HEX` and `AUTONOMIC_MQTT_AUX_PASSWORD` from the generated host file. Source it before `auxiliary.launch`. Do not copy the dashboard or host MQTT passwords to the robot.

For an isolated lab VLAN, the supplied listener uses authenticated MQTT on port 1883 plus application-layer encryption. For any wider network, configure broker certificates in `mosquitto/autonomic.conf`, enable port 8883, and set `tls_ca_cert`/port in `config/mqtt.yaml`.

## Node-RED Dashboard

On the host:

```bash
cd ~/.node-red
npm install node-red-dashboard@3.6.6
```

Merge `node-red/settings-snippet.js` into the `functionGlobalContext` section of `~/.node-red/settings.js`. Do not paste it as a second top-level property if one already exists; add the `crypto` entry to the existing object.

Start Node-RED from the shell where `runtime.env` is sourced, open `http://HOST_IP:1880`, import `node-red/flows.json`, deploy, then open `http://HOST_IP:1880/ui`. If the broker credential does not resolve the environment variable on the installed Node-RED version, edit the broker node once and enter the generated dashboard password.

## Map Creation

Terminal on Raspberry Pi 1:

```bash
roslaunch autonomic_turtlebot3 robot.launch
```

Host terminals:

```bash
roscore
roslaunch autonomic_turtlebot3 mapping.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Drive slowly through every navigable aisle with the lift lowered. Save the result:

```bash
mkdir -p "$HOME/maps"
rosrun map_server map_saver -f "$HOME/maps/factory"
```

## Mission Calibration

Copy `missions/factory_template.yaml` to a site-specific file. Keep `calibrated: false` while editing.

For every named station:

1. Put the robot at the desired approach/dock pose.
2. Read the pose from RViz or `rostopic echo /amcl_pose`.
3. Enter `x`, `y`, and yaw in radians.
4. Confirm the pose with an unloaded navigation goal.
5. Verify fork clearance and lift state.

Update the navigation footprint after measuring the actual platform. Only then set `calibrated: true`.

## Physical Launch Order

1. Host: start the ROS master and source runtime secrets.
2. Raspberry Pi 1: start base mobility and sensing.
3. Raspberry Pi 2: start lift and battery nodes.
4. Host: start mapped navigation and MAPE-K supervision.

```bash
# Host
roscore

# Raspberry Pi 1
roslaunch autonomic_turtlebot3 robot.launch

# Raspberry Pi 2
roslaunch autonomic_turtlebot3 auxiliary.launch mqtt_host:=192.168.10.10

# Host, after all topics and TF are present
roslaunch autonomic_turtlebot3 host_navigation.launch \
  map_file:="$HOME/maps/factory.yaml" \
  mission_file:=/absolute/path/to/factory_calibrated.yaml
```

The physical auxiliary launch defaults to encrypted MQTT for commands and telemetry, matching the paper. For bench diagnosis only, direct ROS transport remains available with `mqtt_transport:=false`; in that mode launch the host with `mqtt_enabled:=false` or expect both transports to be visible.

For the paper's online-SLAM behavior instead of a saved map:

```bash
roslaunch autonomic_turtlebot3 host_slam.launch \
  mission_file:=/absolute/path/to/factory_calibrated.yaml
```

Start one transport cycle from Node-RED or:

```bash
rosservice call /autonomic_manager/start_mission
```

When the pallet assembly is complete, press **Assembly ready** in Node-RED. The calibrated physical mission waits for this encrypted factory signal before collecting the assembled pallet.

Cancel at any time:

```bash
rosservice call /autonomic_manager/cancel_mission
```

After physically clearing a detected stuck condition:

```bash
rosservice call /autonomic_manager/acknowledge_recovery
```

## Simulation Without MQTT

```bash
roslaunch autonomic_turtlebot3 simulation.launch mqtt_enabled:=false
rosservice call /autonomic_manager/start_mission
```

To exercise the complete dashboard path, source the runtime environment, start Mosquitto/Node-RED, and use `mqtt_enabled:=true`.

## Runtime Checks

```bash
rostopic hz /scan /odom /battery_state
rosrun tf tf_echo map base_footprint
rostopic echo /autonomic/state
rostopic echo /autonomic/mapek_event
rosparam get /move_base/base_global_planner
```

The last command must return `autonomic_turtlebot3/RRTGlobalPlanner`. The SQLite knowledge database defaults to `~/.ros/autonomic_turtlebot3/knowledge.db`.
