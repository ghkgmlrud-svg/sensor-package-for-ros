# ISRO-P2 ROS2 Package

ROS2 package for receiving ISRO-P2 PIMTP binary serial data and publishing PVA/GNSS/attitude topics.

## Package and Node

| Item | Name |
| --- | --- |
| ROS2 package | `isro_p2_pkg` |
| Node executable | `isro_p2_node` |
| Python node file | `isro_p2_pkg/isro_p2_node.py` |
| Example config | `config/config_0402.txt` |

## What This Node Reads

The node looks for PIMTP frames with this sync word:

```text
AC 55 96 83
```

Current message IDs:

| Message | ID | Use |
| --- | --- | --- |
| PVA | `2379` | Position, velocity, roll, pitch, yaw |
| IMU | `2389` | Parser exists in code, but publishing currently uses PVA data |

The node validates CRC when possible and can log CRC warnings with `log_crc_warnings:=true`.

## Run

```bash
ros2 run isro_p2_pkg isro_p2_node
```

Run with common parameters:

```bash
ros2 run isro_p2_pkg isro_p2_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=460800 \
  -p poll_hz:=100.0
```

Send the included example sensor configuration before reading data:

```bash
ros2 run isro_p2_pkg isro_p2_node --ros-args -p send_config:=true
```

Use a different config file:

```bash
ros2 run isro_p2_pkg isro_p2_node --ros-args \
  -p send_config:=true \
  -p config_path:=/path/to/config_0402.txt
```

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | Serial device path |
| `baudrate` | `460800` | Serial baudrate |
| `timeout` | `0.02` | Serial read timeout in seconds |
| `poll_hz` | `100.0` | Timer rate used by the ROS node |
| `frame_id` | `isro_p2_link` | Frame ID for velocity and attitude messages |
| `gps_frame_id` | `gps` | Frame ID for GNSS fix messages |
| `send_config` | `false` | Send config commands at startup |
| `config_path` | packaged `config/config_0402.txt` | Config file used when `send_config` is true |
| `log_crc_warnings` | `false` | Log CRC mismatch warnings |

## Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/pva/raw` | `std_msgs/msg/String` | Human-readable parsed PVA summary |
| `/pva/fix` | `sensor_msgs/msg/NavSatFix` | Latitude, longitude, altitude |
| `/pva/velocity` | `geometry_msgs/msg/TwistStamped` | X/Y/Z velocity in m/s |
| `/pva/imu` | `sensor_msgs/msg/Imu` | Orientation quaternion converted from roll/pitch/yaw |

## Data Mapping

The PVA parser tries several known payload layouts and chooses the one that produces realistic values.

Published value assumptions:

```text
latitude, longitude -> degrees
height              -> meters
velocity_x/y/z      -> meters per second
roll, pitch, yaw    -> degrees
```

`/pva/imu` converts roll, pitch, and yaw to quaternion orientation. It does not publish angular velocity or linear acceleration from the IMU message yet.

## Example Checks

Check the serial device:

```bash
ls /dev/ttyUSB*
```

Check published topics:

```bash
ros2 topic list
ros2 topic echo /pva/raw
ros2 topic echo /pva/fix
ros2 topic echo /pva/velocity
ros2 topic echo /pva/imu
```

## Troubleshooting

- If the node cannot open the port, check the `port` parameter and serial permissions.
- If no data appears, check cable, power, baudrate, and whether the sensor is outputting PIMTP.
- If values look wrong, the PVA payload layout may differ from the current assumptions in `parse_pva()`.
- If configuration should be sent to the sensor, set `send_config:=true`; otherwise the node only reads incoming serial data.
