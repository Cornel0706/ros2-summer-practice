## How to Run the Simulation

1. Launch the drone simulation:
   * **With GUI (Standard)**:
     ```bash
     ros2 launch ardrone_gazebo single_ardrone.launch.py
     ```
     * **Launch in Rubicon World**:
     ```bash
     ros2 launch ardrone_gazebo single_ardrone.launch.py world:=rubicon.sdf
     ```
     * **Launch in Empty World**:
     ```bash
     ros2 launch ardrone_gazebo single_ardrone.launch.py world:=ardrone_empty.sdf
     ```
3. Take off:
   ```bash
   ros2 topic pub -1 ardrone/takeoff std_msgs/msg/Empty {}
   ```
4. Control flight commands via teleop keyboard:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
5. Land:
   ```bash
   ros2 topic pub -1 ardrone/land std_msgs/msg/Empty {}
   ```

## Autonomous Visual SLAM, Exploration & Virtual Battery

This mode launches the drone in a custom depot environment, builds a 2D map using the onboard RGBD camera via RTAB-Map, and autonomously explores the unknown areas using MRTSP and Nav2.

1. **Launch the fully autonomous system**:
   ```bash
   ros2 launch ardrone_gazebo visual_slam_drone.launch.py
   ```
   *Note: This automatically spawns the drone in `tugbot_depot.sdf`, opens RViz2 with the mapping configuration, and starts an `rqt_image_view` window for the drone's live camera feed.*

2. **Wait for Auto-Takeoff**:
   The drone will take off automatically after 8 seconds of initialization. Once airborne, the MRTSP exploration node will take control and autonomously navigate the drone to map the environment.

3. **Smart Virtual Battery & Dynamic Return-to-Home (RTH)**:
   This mode includes a built-in virtual battery system with an intelligent, distance-aware RTH algorithm:
   * The battery drains at 1% every 3 seconds (~5 minutes of flight time).
   * The RTH threshold is **calculated dynamically** based on the drone's real-time distance to home: `threshold = (distance / speed / drain_rate) + 5% safety margin`, with a minimum floor of 10%.
   * When battery meets the dynamic threshold, the system gracefully preempts exploration and autonomously flies back to the origin coordinates `[0.0, 0.0]`.
   * It lands automatically and executes a **15-second recharging pit-stop**.
   * Once recharged to 100%, it automatically takes off, resets its flight controllers, and seamlessly resumes mapping the unexplored frontiers.

## Multi-Drone Autonomous Visual SLAM & Exploration

This mode launches two drones simultaneously in a custom depot environment. Both drones build individual 2D maps using their onboard RGBD cameras via RTAB-Map, which are dynamically merged into a single global map. The drones autonomously explore the unknown areas collaboratively using MRTSP and Nav2.

1. **Launch the multi-drone autonomous system**:
   ```bash
   ros2 launch ardrone_gazebo multi_drone_slam.launch.py
   ```
   *Note: This automatically spawns two drones (`drone1` and `drone2`) in `tugbot_depot.sdf`, opens a shared RViz2 configuration with the merged mapping data, and starts `rqt_image_view` windows for each drone's live camera feed.*

2. **Wait for Auto-Takeoff**:
   Both drones will take off automatically after a short initialization delay. Once airborne, the MRTSP exploration nodes will take control and autonomously navigate both drones to collaboratively map the environment.

3. **Smart Virtual Battery & Dynamic Return-to-Home (RTH)**:
   This mode includes the built-in virtual battery system with intelligent, distance-aware RTH algorithm independently running on each drone:
   * The drones independently manage their battery levels and will preempt exploration to fly back to their respective starting coordinates when their dynamic threshold is reached.
   * After landing and recharging for 15 seconds, they automatically take off and resume collaborative mapping.


## Manual Exploration using TELEOP:

This mode launches the drone in a custom depot environment, builds a 2D map using the onboard RGBD camera via RTAB-Map, and u can manually explore the areas.

1. **Launch the manual system**:
   ```bash
   ros2 launch ardrone_gazebo manual_teleop_drone.launch.py
   ```
   *Note: This automatically spawns the drone in `tugbot_depot.sdf`, opens RViz2 with the mapping configuration, and starts an `rqt_image_view` window for the drone's live camera feed.*

2. **Wait for Auto-Takeoff**:
   The drone will take off automatically after 8 seconds of initialization. After that, you can control the drone using TELEOP.

## Heterogeneous Multi-Robot Autonomous Visual SLAM & Exploration (Drone + Ground Robot)

This mode launches an aerial drone (`drone1`) and a ground robot (`lino`) simultaneously in the depot environment. Both robots use onboard 3D RGB-D depth cameras to build individual SLAM maps via RTAB-Map, which are merged in real-time into a unified global map `/map`. MRTSP and Nav2 collaboratively navigate both robots.

1. **Launch the autonomous drone + ground robot system**:
   ```bash
   ros2 launch ardrone_gazebo drone_robot_slam.launch.py
   ```
   *Note: Automatically spawns both robots, initializes SLAM, opens RViz2 with the merged map, and starts collaborative frontier exploration after a 10-second stabilization delay.*

## Heterogeneous Multi-Robot Manual SLAM & Exploration (Drone + Ground Robot)

This mode launches both the drone and ground robot with real-time RGB-D Visual SLAM and shared map merging, while opening automated interactive teleop terminals to manually control both robots.

1. **Launch the manual drone + ground robot system**:
   ```bash
   ros2 launch ardrone_gazebo manual_drone_robot_slam.launch.py
   ```
   *Note: Automatically spawns both robots in `tugbot_depot.sdf`, opens RViz2 with the shared map, and launches two popup terminal windows for keyboard teleop (`teleop_drone1` for aerial controls and `teleop_lino` for ground drive).*

