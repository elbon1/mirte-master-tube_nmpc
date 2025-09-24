#!/bin/bash
ros2 topic pub /io/motor/front_left/speed std_msgs/msg/Int32 '{data: 0}' --once
ros2 topic pub /io/motor/front_right/speed std_msgs/msg/Int32 '{data: 0}' --once
ros2 topic pub /io/motor/rear_left/speed std_msgs/msg/Int32 '{data: 0}' --once
ros2 topic pub /io/motor/rear_right/speed std_msgs/msg/Int32 '{data: 0}' --once
