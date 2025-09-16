# 🗺️ Multi-Map Navigation with Wormholes (ROS 2 + Nav2)
This package provides a **ROS 2 action server** that allows a robot to:
- Navigate **within a single map** using Nav2.
- Seamlessly **switch between multiple maps** using predefined **wormholes** stored in an SQLite database.
- Use the standard `NavigateToPose` action interface for goals.


## 📦 Package Structure

multi_map_navigation/
├── maps/
│ ├── room1.yaml
│ ├── room1.pgm
│ ├── room2.yaml
│ └── room2.pgm
├── sql/
│ └── wormholes.db # SQLite database storing wormholes
├── src/
│ └── multi_map_navigation.cpp # Core C++ node
├── CMakeLists.txt
└── package.xml


---

## ⚙️ Database Schema (`wormholes.db`)
The wormhole database defines transitions between maps.



-- Example wormholes between room1 and room2
INSERT INTO wormholes VALUES ('room1', 'room2', 0.156, 0.399, 0.0, -3.03);
INSERT INTO wormholes VALUES ('room2', 'room1', 0.156, 0.399, 0.0,  3.14);

    map_from: Current map

    map_to: Destination map

    (x, y, z, yaw): Robot pose of the wormhole location

🚀 How It Works

    User sends a NavigateToPose goal with header.frame_id = target_map.

    Node checks the current map:

        If same map, forwards goal to Nav2.

        If different map, navigates to wormhole → switches map → continues navigation.

    Map switching is performed using the /map_server/load_map service.

🔧 Dependencies

    ROS 2 Humble (or later)

    nav2_bringup (for Nav2 stack)

    sqlite3 (for wormhole database)

Install SQLite:

sudo apt install libsqlite3-dev

🛠️ Build

cd ~/ros2_ws
colcon build
source install/setup.bash

▶️ Usage
1. Launch Nav2 with map server

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False \
  map:=~/ros2_ws/src/multi_map_navigation/maps/room1.yaml

2. Run the multi-map navigation node

ros2 run multi_map_navigation multi_map_navigation

3. Send navigation goal to another map

ros2 action send_goal /multi_map_navigation nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'room2'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}}}}"

    Robot will:

        Navigate to wormhole in room1.

        Switch to room2.

        Navigate to (2.0, 3.0) in room2.


