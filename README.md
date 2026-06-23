# Sensor Package for ROS2

ROS2 serial sensor packages for Raspberry Pi / Ubuntu environments.

## Sensors

| Sensor | Folder | Status | Purpose |
| --- | --- | --- | --- |
| MW-AHRS-X1 | `mw_ahrs_x1_pkg` | Working ROS2 package | Serial IMU data to ROS topics |
| ISRO-P2 | `isro_p2_pkg` | Working ROS2 package | PIMTP PVA/GNSS/attitude data to ROS topics |
| Fusion sensor | `fusion_sensor_pkg` | Planned | Sensor-fusion package placeholder |

## Repository Structure

```text
sensor-package-for-ros/
  README.md
  mw_ahrs_x1_pkg/       # ROS2 package for MW-AHRS-X1
  isro_p2_pkg/          # ROS2 package for ISRO-P2
  fusion_sensor_pkg/    # Placeholder for fusion sensor
```

Each sensor folder has its own README with connection notes, parameters, topics, and run examples.

## Tested Environment

- ROS2 Jazzy
- Ubuntu / Raspberry Pi environment
- Python 3
- Serial dependency: `python3-serial` / `pyserial`

## Clone and Build

Clone this repository into a ROS2 workspace `src` directory:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ghkgmlrud-svg/sensor-package-for-ros.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mw_ahrs_x1_pkg isro_p2_pkg
source install/setup.bash
```

Build only one package when needed:

```bash
colcon build --packages-select mw_ahrs_x1_pkg
colcon build --packages-select isro_p2_pkg
```

## Run Examples

MW-AHRS-X1:

```bash
ros2 run mw_ahrs_x1_pkg mw_ahrs_x1_node
```

ISRO-P2:

```bash
ros2 run isro_p2_pkg isro_p2_node
```

## Future Work

- Add launch files if running multiple sensors together becomes common.
- Add parameter YAML files under each package `config/` folder when settings stabilize.
- Add fusion sensor ROS2 node and frame/timestamp assumptions.
