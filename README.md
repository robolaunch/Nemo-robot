# Nemo-robot

### To save map via nav2_map_server
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : '/home/robolaunch/workspaces/nemo-ws/src/mapping/slam/maps/office-v4/map'}"
```

### To save map as serialized map.data via slam_toolbox
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : '/home/robolaunch/workspaces/nemo-ws/src/mapping/slam/maps/office-v4/map'}"
``` 