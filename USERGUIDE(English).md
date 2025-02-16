## Basic Overview

- `cartographer`: A C++ library for SLAM developed by Google.
- `cartographer_ros`: The ROS2 wrapper for cartographer.
- `f1_robot_model`: Provides URDF and SDF files for Gazebo Classic simulation, as well as lua and pbstream files for cartographer simulation.
- `velodyne_simulator`: Offers various plugins and code files to simulate the 3D LiDAR Velodyne in Gazebo Classic.
- Note: All launch files integrate `joy_node` and `teleop_twist_joy` by default (uncomment if needed).

### 1. Installation

```bash
# Step 1.
$ sudo apt update
$ sudo apt install -y \
    cmake g++ git google-mock libceres-dev liblua5.3-dev \
    libprotobuf-dev libsuitesparse-dev libwebp-dev ninja-build \
    protobuf-compiler python3-sphinx libgflags-dev stow
$ git clone https://github.com/kimhoyun-robotair/F1TENTH_Simulation.git
$ cd F1TENTH_Simulation/src/cartographer/scripts
$ ./install_abseil.sh

# Step 2.
$ cd && cd F1TENTH_Simulation
$ colcon build --symlink-install
# It takes a few minutes
$ source install/setup.bash
# or source install/local_setup.bash
```
### 2. Using Gazebo Classic Simulation
**1. To spawn only the robot in Gazebo:**

```bash
$ ros2 launch f1_robot_model robot_spawn.launch.py
```

**2. To spawn a custom world file (.sdf):**

```bash
$ ros2 launch f1_robot_model display.launch.py
```
- Modify the file in `~/src/f1_robot_model/world` and update the `world_path` in `display.launch.py` to upload and spawn your custom world file.

**3. To automatically publish the odom topic and odom frame from the URDF file:**
- Edit the file `~/src/f1_robot_model/urdf/racecar.gazebo` and update the following section:

```xml
      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_distance>true</publish_distance>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
```
- Note: For cartographer to work correctly, set publish_odom_tf to false, and configure the odom topic as needed.

### 3. Simulating with Cartographer
**1. To perform mapping using cartographer:**

```bash
$ ros2 launch f1_robot_model cartographer.launch.py
```
- You can tune cartographer parameters by editing the `.lua` files in `~/src/f1_robot_model/config`.

**2. To perform localization with cartographer and publish the odom topic and odom TF:**

```bash
$ ros2 launch f1_robot_model localization.launch.py
```
- Similarly, adjust the `.lua` files for tuning.
- Edit the file `~/src/f1_robot_model/urdf/racecar.gazebo` as follows:

```xml
      <!-- output -->
      <publish_odom>false</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_distance>true</publish_distance>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
```
### 4. Using AMCL
**1. To use AMCL:**
```bash
$ ros2 launch f1_robot_model amcl.launch.py
```
- The AMCL configuration and demomap files are located in `~/f1_robot_model/map` and `~/f1_robot_model/config`.
