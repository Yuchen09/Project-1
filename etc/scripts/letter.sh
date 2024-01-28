#!/bin/bash

##  make sure run command ros2 run turtlesim turtlesim_node on another termial first!!!  ##

# first kill the initial turtle1
ros2 service call /reset std_srvs/srv/Empty
ros2 service call /kill turtlesim/srv/Kill "{name: turtle1}"

# generate two turtles called Y and U
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1, y: 8, theta: 0, name: Y }"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 6, y: 8, theta: 0, name: U }"

#set colors
ros2 service call /Y/set_pen turtlesim/srv/SetPen "{r: 100, g: 125, b: 34, width: 5}"
ros2 service call /U/set_pen turtlesim/srv/SetPen "{r: 200, g: 250, b: 100, width: 5}"

# draw Y
ros2 service call /Y/teleport_absolute turtlesim/srv/TeleportAbsolute " {x: 3, y: 6, theta: 315}"
ros2 service call /Y/teleport_absolute turtlesim/srv/TeleportAbsolute " {x: 3, y: 1.5, theta: -45}"
ros2 service call /Y/teleport_absolute turtlesim/srv/TeleportAbsolute " {x: 3, y: 6, theta: -180}"
ros2 service call /Y/teleport_absolute turtlesim/srv/TeleportAbsolute " {x: 5, y: 8, theta: 45}"

# draw U
ros2 service call /U/teleport_absolute turtlesim/srv/TeleportAbsolute " {x: 6, y: 3, theta: 270}"
ros2 topic pub --once /U/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0 , y: -5 , z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.14}}"
sleep 0.5
ros2 service call /U/teleport_absolute turtlesim/srv/TeleportAbsolute " {x: 9.2, y: 8, theta: 90}"

# remove the turtles to make letters more clearly
ros2 service call /kill turtlesim/srv/Kill "{name: Y}"
ros2 service call /kill turtlesim/srv/Kill "{name: U}"
