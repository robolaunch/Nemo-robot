# Nav2 Bringup 

### To give initial pose without Rviz2 (example)
```bash
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: -0.25, y: -0.9, z: 0.0}, orientation: {w: 0.1}}, } }'
```

### To start mapping via Nav2 Stack
```bash
ros2 launch bringup_robolaunch mapping.launch.py
```

### To start navigation via Nav2 Stack (We need to define our robot urdf file)
```bash
ros2 launch bringup_robolaunch bringup_robot.launch.py
```


### Nav2 bringup briefs of parameters
```bash
https://navigation.ros.org/configuration/packages/configuring-amcl.html
```

# slam
