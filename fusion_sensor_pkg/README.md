# Fusion Sensor ROS2 Package

Status: planned.

This folder is reserved for the fusion sensor package. It is not a buildable ROS2 package yet because the node implementation has not been added.

## Information to Add

When implementing this sensor, document:

- Which sensors are fused
- Input topics or input serial devices
- Fusion algorithm or data source
- Coordinate frame assumptions
- Timestamp assumptions
- ROS2 package name and node executable name
- Parameters
- Published topics and message types
- Example `ros2 run` command
- Example `ros2 topic echo` command

## Recommended Future Structure

```text
fusion_sensor_pkg/
  README.md
  package.xml
  setup.py
  setup.cfg
  resource/fusion_sensor_pkg
  fusion_sensor_pkg/
    __init__.py
    fusion_sensor_node.py
```
