# mbot_slam

This package contains three nodes: **mapping** (odometry-based), **localization** (particle filter on a given map), and **SLAM** (simultaneous mapping + localization). Shared algorithms live in `include/common/` and `src/common/`.

## Odometry-based Mapping

Odometry-based occupancy grid mapping. Subscribes to `/scan` and odometry tf, publishes `/map`.

### ROS bag testing
**First disconnect the USB cable from pico to pi!!!!**

1. **VSCode Terminal 1** — start the mapping node:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 launch mbot_slam mapping.launch.py
    ```
2. **On NoMachine Terminal:**
    ```
    cd ~/mbot_ros_labs/src/mbot_slam/rviz
    ros2 run rviz2 rviz2 -d mapping.rviz
    ```
3. **VSCode Terminal 2** — Play ROS bag
    ```bash
    cd ~/mbot_ros_labs/src/mbot_rosbags
    ros2 bag play slam_test
    ```

### Real world testing
1. **VSCode Terminal 1** — bring up the robot:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
2. **VSCode Terminal 2** — start the mapping node:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 launch mbot_slam mapping.launch.py
    ```
3. **On NoMachine Terminal:**
    ```
    cd ~/mbot_ros_labs/src/mbot_slam/rviz
    ros2 run rviz2 rviz2 -d mapping.rviz
    ```
4. **VSCode Terminal 3** — Using teleop control to move the robot in the maze
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
5. **VSCode Terminal 4** — save the map when done:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_slam/maps
    ros2 run nav2_map_server map_saver_cli -f your_map_name
    ```

**To view a saved map:**
```bash
ros2 launch mbot_slam view_map.launch.py map_name:=your_map_name
```

## Localization

Particle filter Monte Carlo Localization on a given map.

**For localization, we only do ROS bag testing.**

### ROS bag testing
**First disconnect the USB cable from pico to pi!!!!**

1. **On NoMachine Terminal**:
    ```
    cd ~/mbot_ros_labs/src/mbot_slam/rviz
    ros2 run rviz2 rviz2 -d localization.rviz
    ```
2. **VSCode Terminal 1**:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    # When use rosbag, publish_tf should be false
    ros2 run mbot_slam localization_node --ros-args -p publish_tf:=false
    ```
3. **VSCode Terminal 2**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_rosbags/maze1
    ros2 bag play maze1.mcap
    ```

## SLAM

Particle filter based SLAM.

### ROS bag testing
**First disconnect the USB cable from pico to pi!!!!**

1. **On NoMachine Terminal**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_slam/rviz
    ros2 run rviz2 rviz2 -d slam.rviz
    ```
2. **VSCode Terminal 1**:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_slam slam_node
    ```
3. **VSCode Terminal 2**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_rosbags
    ros2 bag play slam_test
    ```

### Real world testing
1. **On NoMachine Terminal**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_slam/rviz
    ros2 run rviz2 rviz2 -d slam.rviz
    ```
2. **VSCode Terminal 1** — bring up the robot:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
3. **VSCode Terminal 2**:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_slam slam_node
    ```
4. **VSCode Terminal 3** — Using teleop control to move the robot in the maze
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
5. **VSCode Terminal 4** — save the map when done:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_slam/maps
    ros2 run nav2_map_server map_saver_cli -f your_map_name
    ```

**To view a saved map:**
```bash
ros2 launch mbot_slam view_map.launch.py map_name:=your_map_name
```
