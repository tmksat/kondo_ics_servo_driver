# kondo-ics-servo-driver

## Overview

This repository is a driver for controlling servo motors using KONDO's ICS protocol in a ROS2 environment. It includes detailed instructions for building, execution methods, service invocation procedures, and parameter descriptions.

## Tested Environment

- Ubuntu 22.04.1 arm64
- ROS2 Humble
- Tested with [KONDO KRS-5054HV ICS](https://kondo-robot.com/product/03180)
- Communication speed: 115200 bps

## Services

This ROS node provides the following services:

1. `/set_position`
    - **Description**: Sets the angle of the specified servo motor.
    - **Request**: `kondo_ics_servo_msgs/srv/SetPosition`
      - `servo_id` (int): Servo motor ID
      - `angle` (float): Angle to set [rad]
    - **Response**: 
      - `success` (bool): Whether the angle setting was successful

2. `/get_position`
    - **Description**: Gets the current angle of the specified servo motor.
    - **Request**: `kondo_ics_servo_msgs/srv/GetPosition`
      - `servo_id` (int): Servo motor ID
    - **Response**: 
      - `angle` (float): Current angle [rad]

3. `/set_id`
    - **Description**: Sets the servo motor ID.
    - **Request**: `kondo_ics_servo_msgs/srv/SetId`
      - `new_id` (int): New servo motor ID
    - **Response**: 
      - `success` (bool): Whether the ID setting was successful

4. `/get_id`
    - **Description**: Gets the ID of the connected servo motor.
    - **Request**: `kondo_ics_servo_msgs/srv/GetId`
      - Request is an empty message.
    - **Response**: 
      - `servo_id` (int): Servo motor ID

5. `/free`
    - **Description**: Turns off the torque of the specified servo motor.
    - **Request**: `kondo_ics_servo_driver/srv/Free`
      - `servo_id` (int): Servo motor ID
    - **Response**: 
      - `success` (bool): Whether the torque off was successful

## Parameters

The following parameters can be configured for this ROS node:

1. `port`
    - **Description**: Specifies the serial port device file for communicating with the servo motor.
    - **Default value**: `/dev/ttyUSB0`
    - **Example usage**: `/dev/ttyUSB1`

These parameters can be specified when launching the node using the format `--ros-args -p <parameter_name>:=<value>`.

## Build

1. Navigate to your workspace.
2. Build the package.
3. Set up environment variables.
```shell
$ cd ~/ros2_ws
$ colcon build --packages-select kondo_ics_servo_driver
$ source install/setup.bash
```

## Execution

### Launching the Node

Using the default serial port (/dev/ttyUSB0):
```shell
$ ros2 run kondo_ics_servo_driver kondo_ics_servo_driver_node
```

Specifying a serial port:
```shell
$ ros2 run kondo_ics_servo_driver kondo_ics_servo_driver_node --ros-args -p port:=/dev/ttyUSB1
```

### Service Calls

__Setting servo angle (ID:1 servo to 45 degrees = 0.785 rad)__
```shell
$ ros2 service call /set_position kondo_ics_servo_msgs/srv/SetPosition "{serve_id: 1, angle: 0.785}"
```

__Getting current angle (ID:1 servo)__
```shell
$ ros2 service call /get_position kondo_ics_servo_msgs/srv/GetPosition "{serve_id: 1}"
```

__Setting servo ID (to ID:2)__
```shell
$ ros2 service call /set_id kondo_ics_servo_msgs/srv/SetId "{new_id: 2}"
```

__Getting current servo ID (please connect only one servo)__
```shell
$ ros2 service call /get_id kondo_ics_servo_msgs/srv/GetId "{}"
```

__Turning off servo torque (ID:1)__
```shell
$ ros2 service call /free kondo_ics_servo_driver/srv/Free "{servo_id: 1}"
```
