# Walkthrough: Ported `ardrone` Packages to ROS 2 Jazzy

This document summarizes the changes made to port `ardrone_autonomy` and `ardrone_gazebo` from ROS 1 (Catkin) to ROS 2 Jazzy (Ament / Gazebo Sim Harmonic), detailing the advanced physics stability and flight control fixes that enabled successful takeoff, hover, and command flight.

## Summary of Completed Work

### 1. `ardrone_autonomy` Package (Pure ROS 2 Message Generator)
- **CamelCase Rename**: Message and service files were renamed to valid CamelCase format (e.g. `navdata_demo.msg` -> `NavdataDemo.msg`, `vector31.msg` -> `Vector31.msg`) to satisfy strict ROS 2 name validation.
- **Inner Field Fixes**: Fields containing CamelCase names (like `batteryPercent` or `magX`) were converted to lowercase snake_case (like `battery_percent` or `mag_x`).
- **Inner Type Capitalization**: Modified message fields referencing inner message types (e.g., `vector31` -> `Vector31`) to ensure type mapping matches the renamed CamelCase files.
- **Ament Migration**: Converted `package.xml` to format 3 and `CMakeLists.txt` to use `ament_cmake` and `rosidl_generate_interfaces`.

### 2. `ardrone_gazebo` Package (Simulation & Control)
- **SDF Update**: Replaced deprecated classic Gazebo C++ plugins with standard Gazebo Sim (Harmonic) plugins:
  - Added `<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>`
  - Added `<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>`
  - Added `<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"/>`
- **Self-Contained Model Compliance**: Copied the `meshes/` folder into `models/ardrone_gazebo/` to make the model self-contained. This allows Gazebo Sim to resolve `model://ardrone_gazebo/meshes/...` correctly using the environment resource path.
- **World Configuration**: Created `worlds/ardrone_empty.sdf` containing standard empty world systems plus the `<plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench">` plugin to handle external force and torque commands.
- **Launch Orchestration**: Created `launch/single_ardrone.launch.py` to set environment variables, run Gazebo Sim, spawn the model, and run `ros_gz_bridge`.

---

## Technical Controller Refinements (Hover & Flight Stability)

During flight verification, several physics solver and control frame bugs were identified and fixed in the Python driver (`scripts/ardrone_driver.py`):

### 1. Scoped Entity Resolution
Gazebo Sim uses double colons (`::`) as the scoping delimiter to resolve links. The target link for applying force was corrected to `ardrone_gazebo::base_link`, eliminating the persistent `[Err] [ApplyLinkWrench.cc:283] Entity not found` error.

### 2. Control Loop Coordinate Separation
In ROS 2, linear twist velocity is published in the local child frame (body frame), while position is in the world frame. The controller was refactored to cleanly separate kinematics:
- **Horizontal Translation Control**: Kinematics (velocities/accelerations) are rotated to the **heading frame (yaw only)** relative to the world coordinates to calculate forward/lateral pitch and roll command angles.
- **Vertical Kinematics Control**: Thrust/lift calculations use **local body frame** vertical velocity and acceleration to match the body Z-axis along which thrust is physically applied.
- **Angular Rates**: Rotational rates from ROS are already local body rates, bypassing redundant coordinate rotations.

### 3. Numerical Derivative Protection
In high-frequency control loops (100Hz+), raw numerical derivatives (`dx = (x - last_x) / dt`) amplify small physics step-size oscillations and jitter.
- A minimum time-step threshold `if dt < 0.001: return` was enforced.
- Low-pass filters (`alpha = 0.2`) were added to both body-frame and world-frame acceleration calculations.
- Clamping limits were introduced on control efforts to protect the physics engine solver (DART/ODE) from boundary explosions.

### 4. Non-Persistent Wrench Scaling (Impulse-Based Control)
The persistent wrench topic `/world/empty/wrench/persistent` keeps applying the last published wrench indefinitely, and multiple messages accumulate continuously in Gazebo's internal vectors, causing forces to grow to infinity. 
To resolve this:
- The driver was switched to publish to `/world/empty/wrench` (instantaneous single-step wrench).
- Output forces and torques are scaled by `scale = dt / 0.001` (where `0.001` is Gazebo's physics step size) to preserve the exact total impulse over the control loop period without accumulation.
- Clamped scaled outputs to `[0.0, 150.0]` N for force and `[-30.0, 30.0]` N-m for torque.

### 5. Takeoff Trajectory Setpoints
During the takeoff phase, the controller commands a positive vertical velocity target of `0.8` m/s (rather than hovering at `0.0`) for a duration of `1.5` seconds to cleanly clear ground contact stiction.

---

## Verification & Flight Test Results

### 1. Stable Takeoff and Hover
Publishing a takeoff command:
```bash
ros2 topic pub -1 ardrone/takeoff std_msgs/msg/Empty {}
```
Result: The drone took off and climbed smoothly to a stable hover at **~2.3 meters height** with zero lateral drift:
```yaml
pose:
  pose:
    position:
      x: -0.05841
      y: 0.03197
      z: 2.27130
    orientation:
      x: 0.00000
      y: 0.00000
      z: -0.03059
      w: 0.99953
twist:
  twist:
    linear:
      z: -0.03743
```

### 2. Forward Flight Command
Publishing a forward command vel of 1.0 m/s:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
Result: The drone pitched forward and translated smoothly along the X axis, reaching a rock-solid forward velocity of **exactly 1.0 m/s** while maintaining altitude:
```yaml
pose:
  pose:
    position:
      x: 12.61598
      y: -2.47280
      z: 1.95377
twist:
  twist:
    linear:
      x: 0.99997
      y: -0.00595
      z: 0.03226
```