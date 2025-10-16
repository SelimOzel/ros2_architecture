# ROS 2 Mini System: Proximity Speed Control with Emergency Stop

This repository contains three ROS 2 (Humble) C++ nodes that demonstrate a simple control pipeline:

- `ProximitySensorNode`: publishes a synthetic proximity measurement (in mm) as a sine wave.
- `EmergencyStopNode`: toggles an emergency-stop boolean via keyboard input (`E/e` in a terminal).
- `SpeedControlNode`: computes a max-speed mode string (`STOP`, `SLOW`, `FULL_SPEED`) from proximity, with E-stop override.

## System Architecture
                          (std_msgs/Float64)                                     (std_msgs/String)
    +-------------------+  "proximity_sensor"  @10 Hz        +-------------------+   "max_speed"   @on-change   +------------------------+
    | ProximitySensor   | ----------------------------------> |   SpeedControl    | ---------------------------> |  Consumers (optional)  |
    |       Node        |                                     |       Node        |                              |  e.g., motion planner  |
    +-------------------+                                     +-------------------+                              +------------------------+
                                                              ^ 
                                                              |
                                              (std_msgs/Bool) |
    +-------------------+  "emergency_stop"  @on-change  -----+
    |  EmergencyStop    |
    |       Node        |
    +-------------------+

Legend:
- Topic name in quotes, message type in parentheses.
- `@10 Hz` etc. indicate expected publish rates. `@on-change` means only when state changes.
- Arrows show message flow direction.

## Design Decisions
We use a ROS 2 multi-node architecture with standard publishers/subscribers. ProximitySensorNode and EmergencyStopNode each publish to /proximity_sensor and /emergency_stop, respectively. SpeedControlNode subscribes to both.

ProximitySensorNode publishes a synthetic proximity value at 10 Hz using an internal sine-wave generator (default: amplitude = 1200 mm, frequency = 0.5 Hz). The node is implemented as a single source file to keep scope small.

EmergencyStopNode polls keyboard input (E/e) at 20 Hz and publishes only on state changes (edge-triggered).

SpeedControlNode applies simple, deterministic rules: FULL_SPEED when no object is within > 800 mm, SLOW for 400–800 mm, and STOP for < 400 mm. An active E-stop overrides all proximity logic. The node publishes /max_speed only when the computed state changes.

Attribution: Most of the code is adapted from ROS 2 tutorials. StdinRawMode was generated with ChatGPT v5.0 to capture key presses.

## Developer Guide
### a. Install ROS 2 (Humble)
Follow OS-specific instructions from ROS 2 docs [1] (brief commands below).

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### b. Node Descriptions
Based on [2], the proximity and emergency-stop nodes follow the standard ROS 2 publisher example. The speed-control node subscribes to both topics and applies the control logic.

### c. Build Instructions
```bash
cd <PATH_TO>/ros2_architecture_project
conda deactivate
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build
source install/setup.bash
ros2 run proximity_sensor_node proximity_sensor --ros-args -p frequency_hz:=0.1 -p amplitude_mm:=1200.0
ros2 run emergency_stop_node emergency_stop
ros2 run speed_control_node speed_control
```

### d. References
[1] https://docs.ros.org/en/humble/Installation.html

[2] https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

## Testing
Run each node in separate terminals using ros2 run:
1. proximity_sensor_node → proximity_sensor
2. emergency_stop_node → emergency_stop
3. speed_control_node → max_speed

Validation steps:
* Node isolation: Start each node independently to confirm it launches without errors.
* Proximity → Speed: Run proximity + speed control. Observe max_speed transitions across FULL_SPEED (≥800 mm), SLOW (400–800 mm), and STOP (<400 mm) as the sine wave evolves.
* E-stop override: Run emergency stop + speed control. Toggle E/e and verify max_speed holds STOP while E-stop is active, regardless of proximity.
* Full pipeline: Run all three. Confirm E-stop forces STOP; when released, proximity again drives the state.

Expected behavior: speed_control_node publishes max_speed only on state changes (edge-triggered).