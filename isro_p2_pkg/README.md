# ISRO-P2 ROS2 Package

Status: planned.

This folder is reserved for the ISRO-P2 sensor package. It is not a buildable ROS2 package yet because the node implementation has not been added.

## Information to Add

When implementing this sensor, document:

- Sensor model and purpose
- Connection method, such as USB, UART, or another interface
- Serial port or device path
- Baudrate and communication settings
- Startup command, if the sensor needs one
- ROS2 package name and node executable name
- Parameters
- Published topics and message types
- Data order and units
- Timestamp source
- Example `ros2 run` command
- Example `ros2 topic echo` command

## Recommended Future Structure

```text
isro_p2_pkg/
  README.md
  package.xml
  setup.py
  setup.cfg
  resource/isro_p2_pkg
  isro_p2_pkg/
    __init__.py
    isro_p2_node.py
```
