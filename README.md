# Sensor Package for ROS2

This repository contains ROS2 packages and notes for serial sensor data acquisition.

## Sensors

| Sensor | Folder | Status | Purpose |
| --- | --- | --- | --- |
| MW-AHRS-X1 | `mw_ahrs_x1_pkg` | Working ROS2 package | Serial IMU data to ROS topics |
| ISRO-P2 | `isro_p2_pkg` | Planned | Sensor package placeholder |
| Fusion sensor | `fusion_sensor_pkg` | Planned | Sensor-fusion package placeholder |

## Repository Structure

```text
sensor-package-for-ros/
  README.md
  .gitignore
  mw_ahrs_x1_pkg/       # ROS2 package for MW-AHRS-X1
  isro_p2_pkg/          # Placeholder for ISRO-P2
  fusion_sensor_pkg/    # Placeholder for fusion sensor
```

Each sensor has its own folder and README. When a sensor is implemented, its folder should be a normal ROS2 package with `package.xml`, `setup.py`, source code, parameters, and topic documentation.

## Tested Environment

- ROS2 Jazzy
- Ubuntu / Raspberry Pi environment
- Python 3
- Serial dependency: `python3-serial` / `pyserial`

Install the serial dependency if needed:

```bash
sudo apt install python3-serial
```

## Clone and Build

Clone this repository into a ROS2 workspace `src` directory:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ghkgmlrud-svg/sensor-package-for-ros.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mw_ahrs_x1_pkg
source install/setup.bash
```

You can also use `local_setup.bash` after sourcing ROS2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

## Current Working Package

### MW-AHRS-X1

Run:

```bash
ros2 run mw_ahrs_x1_pkg mw_ahrs_x1_node
```

Check topics:

```bash
ros2 topic list
ros2 topic echo /imu/raw
ros2 topic echo /imu/data
ros2 topic echo /imu
```

The node automatically sends `+++` and then `ss=4` to the sensor at startup.

## Common Notes for Users

Before running a sensor node, check these items:

- The sensor is connected to the computer or Raspberry Pi.
- The serial port is correct, for example `/dev/ttyUSB0`.
- The current user has permission to access the serial device.
- ROS2 environment is sourced.
- The package has been built after cloning or editing.

If serial permission is denied, add the user to the `dialout` group and log out/in:

```bash
sudo usermod -a -G dialout $USER
```

## Documentation Policy

- Root `README.md`: overall repository structure, build method, and sensor list.
- Sensor folder `README.md`: sensor-specific connection, parameters, topics, and run examples.
- Code comments: only for logic that is not obvious from the code.

## Future Work

- Add ISRO-P2 ROS2 node and topic mapping.
- Add fusion sensor ROS2 node and frame/timestamp assumptions.
- Add launch files if running multiple sensors together becomes common.
- Add parameter YAML files under each package `config/` folder when settings stabilize.
