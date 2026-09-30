# Hardware Build

## Bill of Materials

### TurtleBot3 Mobility and Sensing

- TurtleBot3 Waffle Pi chassis and plates
- OpenCR 1.0 controller
- 2 x Dynamixel XL430 wheel actuators and Waffle Pi wheels/caster
- Raspberry Pi used by the standard Waffle Pi setup (Raspberry Pi 3B+ in the original platform; a compatible Noetic SBC may be used)
- TurtleBot3 LDS LiDAR, IMU, wheel encoders, cabling, and OpenCR power path

### Lift and Energy Additions

- Second Raspberry Pi with Wi-Fi/Ethernet, microSD, and a regulated 5 V supply rated for its peak current
- 12.8 V nominal, 4-cell LiFePO4 battery with integrated BMS and appropriate charger
- Main fuse close to the battery, branch fuses, latching hardware emergency stop, disconnect, and covered distribution terminals
- DC/DC converters. The paper names LM2596 modules; use a higher-current, low-ripple 5 V converter for each Raspberry Pi if an LM2596 module cannot meet measured peak demand
- INA219-compatible high-side monitor/shunt rated for expected current, or replace the battery node with the actual BMS telemetry interface
- Stepper motor(s), lead screw/belt lift, and load-rated guides
- One TB6600-class driver per independently driven stepper. If two lift motors move together, use two drivers sharing STEP/DIR or a mechanically synchronized single motor; do not assume two motors can safely share one driver
- Two normally-closed limit switches, one at each travel endpoint
- 3.3 V-compatible opto/level interface between Raspberry Pi GPIO and TB6600 STEP/DIR/ENABLE inputs
- Rear battery enclosure/counterweight, front lift support, forks/claws, fasteners, wire guards, and mechanical end stops
- Pallets sized for the finished forks

The paper does not publish CAD files, dimensions, payload mass, center-of-gravity limits, motor torque, or transmission details. Those must be engineered and validated for the chosen payload. Do not infer load capacity from the TurtleBot3 chassis.

## Raspberry Pi 2 Signal Assignment

BCM numbering is used by `config/lift.yaml`.

| BCM pin | Direction | Connection | Required behavior |
|---:|---|---|---|
| GPIO18 | Output | STEP inputs through level/opto interface | One pulse per configured microstep |
| GPIO23 | Output | DIR inputs through level/opto interface | Polarity calibrated before coupling the load |
| GPIO24 | Output | ENABLE inputs through level/opto interface | Active-low by default |
| GPIO17 | Input | Upper normally-closed limit switch | Opens at upper endpoint |
| GPIO27 | Input | Lower normally-closed limit switch | Opens at lower endpoint |
| GPIO2/3 | I2C | INA219 SDA/SCL | Battery telemetry at address `0x40` |
| GND | Reference | Signal-interface ground | Common only as required by the selected isolated/interface circuit |

Normally-closed switches are intentional: a broken wire reads as a stop condition. Keep motor wiring away from LiDAR, encoder, I2C, and network wiring. Fit flyback/suppression and shielding according to driver and converter manufacturers.

## Power Topology

```text
LiFePO4 battery + BMS
        |
  main fuse + disconnect + E-stop
        |
        +-- fused OpenCR / wheel branch
        +-- fused TB6600 motor-power branch
        +-- fused regulated 5 V -> Raspberry Pi 1
        +-- fused regulated 5 V -> Raspberry Pi 2
        +-- INA219/BMS telemetry -> Raspberry Pi 2
```

The hardware emergency stop must remove drive energy independently of ROS, Wi-Fi, Linux, and GPIO. Do not route motor current through a breadboard or the Raspberry Pi.

## Mechanical Integration

- Keep the LiDAR's full scan plane unobstructed by the rear battery, mast, payload, or cables.
- Mount the battery low and rearward as the paper's counterweight, then verify stability at maximum lift height and during braking.
- Ensure the forks do not contact the floor throughout caster travel.
- Add physical end stops beyond the electrical limits.
- Guard lead screws, belts, pinch points, and stepper couplings.
- Make the pallet release fail-safe on power loss.
- Measure the finished footprint at the widest/longest points, including forks. Update `config/costmap_common.yaml`; the supplied polygon is only a conservative starting example.

## Bring-Up Order

1. Power and network-test both Raspberry Pis with motor power disconnected.
2. Verify OpenCR, LiDAR, odometry, and `/cmd_vel` on the unmodified Waffle Pi.
3. Test each limit switch using GPIO reads before connecting STEP.
4. Connect the driver with the lift mechanically unloaded. Set a low current and low step frequency.
5. Verify `up`/`down` polarity, switch action, enable polarity, timeout, and E-stop.
6. Couple the mechanism and repeat with no pallet.
7. Calibrate battery voltage against a trusted meter.
8. Measure the final footprint and map the environment.
9. Increase payload in controlled increments only after structural, thermal, braking, and stability review.

Never test the induced-stuck scenario at full speed or with a raised payload.
