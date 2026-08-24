# mbot_nav

Navigation package with path planning, autonomous exploration, and motion control.
Planners live in `include/planners/`, controllers in `include/controllers/`.


## Controller

We provide implementations of common robotic motion control strategies:
- RTR (Rotate-Translate-Rotate) Controller: A point-to-point state machine, rotating to face the goal, driving straight, and a final rotation to the target orientation.
- Pure Pursuit Controller: A path-following lateral controller. It provides continuous, smooth steering by calculating the circular arc required to reach a "look-ahead" point on a predefined path.
- Example Controller: A template for implementing custom control laws.

Controllers subscribes to `/waypoints`, publishes `/cmd_vel`.

1. Select controllers in `controller_node.cpp`
    ```cpp
    // To switch controllers, change the include and the type of controller_ below.
    #include "controllers/rtr_controller.hpp"
    // #include "controllers/pure_pursuit_controller.hpp"
    // #include "controllers/example_controller.hpp"

    using Controller = RTRController;
    // using Controller = PurePursuitController;
    // using Controller = ExampleController;
    ```
2. Compile, then run the controller node (odom based)
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_nav controller_node
    ```
3. Test with a square trajectory publisher
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_nav square_publisher
    ```
## Navigation

### Test planners offline

Visualize planned path on a saved map without moving the robot.
1. Select planners in `navigation_node.cpp`
    ```cpp
    // To switch planners, change the include and the type of Planner below.
    #include "planners/astar_planner.hpp"
    // #include "planners/theta_star_planner.hpp"
    // #include "planners/example_planner.hpp"

    using Planner = AStarPlanner;
    // using Planner = ThetaStarPlanner;
    // using Planner = ExamplePlanner;
    ```
2. Run the launch file, 
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 launch mbot_nav path_planning.launch.py map_name:=maze1
    ```
3. Open RViz
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```
4. Set **2D Pose Estimate** then **2D Goal Pose** in RViz you should see planned path show up.


### Test navigation (robot follows planned path) by launch file

Robot drives to a goal using a saved map and particle-filter localization.

1. Bring up the mbot:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
2. Launch navigation (map server + localization + planner + controller):
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 launch mbot_nav navigation.launch.py map_name:=your_map
    ```
3. Open RViz, set **2D Pose Estimate** (needed by the particle filter), then **2D Goal Pose**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```

### Test navigation (robot follows planned path) by individual commands
1. Bring up the mbot:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
2. Launch the map server:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 launch mbot_nav path_planning.launch.py map_name:=your_map pose_source:=tf
    ```
3. Run the localization node:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_slam localization_node --ros-args -p publish_tf:=true
    ```
4. Run the controller:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_nav controller_node --ros-args -p use_localization:=true
    ```
5. Open RViz, set **2D Pose Estimate**, then **2D Goal Pose**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```


## Exploration (robot maps and explores autonomously)

Robot builds the map with SLAM while navigating to frontier goals. Frontier Explorer using a Wavefront-style Breadth-First Search (BFS).

### Run by launch file
1. Bring up the mbot:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
2. Launch exploration (SLAM + planner + controller + frontier explorer):
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 launch mbot_nav exploration.launch.py
    ```
3. Open RViz to watch the map grow:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```

### Run by individual commands
1. Bring up the mbot:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
2. Run the SLAM node:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_slam slam_node
    ```
3. Run the navigation node (TF mode, SLAM provides the pose):
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_nav navigation_node --ros-args -p pose_source:=tf
    ```
4. Run the controller:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_nav controller_node --ros-args -p use_localization:=true
    ```
5. Run the exploration node:
    ```bash
    cd ~/mbot_ros_labs
    source install/setup.bash
    ros2 run mbot_nav exploration_node
    ```
6. Open RViz to watch the map grow:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```
