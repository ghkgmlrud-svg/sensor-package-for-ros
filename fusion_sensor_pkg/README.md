# Fusion Sensor ROS2 Package

ROS2 package for reading ISRO-P2 PVA/GNSS data and MW-AHRS-X1 IMU data, then publishing each sensor stream from one node.

## Package and Node

| Item | Name |
| --- | --- |
| ROS2 package | `fusion_sensor_pkg` |
| Node executable | `fusion_sensor_node` |
| Python node file | `fusion_sensor_pkg/fusion_sensor_node.py` |
| Pre-ROS test | `pre_ros_test/fusion_sensor_test.py` |

## Run

```bash
ros2 run fusion_sensor_pkg fusion_sensor_node
```

Run with common parameters:

```bash
ros2 run fusion_sensor_pkg fusion_sensor_node --ros-args \
  -p isro_port:=/dev/ttyUSB0 \
  -p isro_baudrate:=460800 \
  -p mw_port:=/dev/ttyUSB1 \
  -p mw_baudrate:=115200
```

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `isro_port` | `/dev/ttyUSB0` | ISRO-P2 serial device path |
| `isro_baudrate` | `460800` | ISRO-P2 serial baudrate |
| `mw_port` | `/dev/ttyUSB1` | MW-AHRS-X1 serial device path |
| `mw_baudrate` | `115200` | MW-AHRS-X1 serial baudrate |

## Pre-ROS Test

```bash
python3 fusion_sensor_pkg/pre_ros_test/fusion_sensor_test.py
```

Use the script before launching ROS2 when checking raw serial parsing or port assignments.
