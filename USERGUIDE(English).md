## Basic Guide

- **`f1_robot_model`**: Provides `URDF` and `SDF` files for Gazebo Classic simulation, as well as `lua` and `pbstream` files for Cartographer simulation.
- **`cartographer_ros`**: Offers SLAM functionality for Mapping and Localization.

### 1. Installation Steps
```bash
$ git clone 
$ cd 
$ colcon build --symlink-install
$ source install/setup.bash
# source install/local_setup.bash
```

### 2. How to Use Gazebo Classic Simulation

1. **To spawn only the robot in Gazebo:**
   ```bash
   $ ros2 launch f1_robot_model robot_spawn.launch.py
   ```

2. **To spawn a custom world file (i.e., your own `.sdf` file):**
   ```bash
   $ ros2 launch f1_robot_model display.launch.py
   ```
   - You can upload and spawn a custom world file by modifying the files within `~/src/f1_robot_model/world` and the `world_path` section in the `display.launch.py` file.

3. **To automatically publish the odom topic and odom frame from the URDF file:**
   - Modify the code inside `~/src/f1_robot_model/urdf/racecar.gazebo` as follows:
     ```xml
          <!-- output -->
          <publish_odom>true</publish_odom>
          <publish_odom_tf>true</publish_odom_tf>
          <publish_wheel_tf>true</publish_wheel_tf>
          <publish_distance>true</publish_distance>

          <odometry_frame>odom</odometry_frame>
          <robot_base_frame>base_link</robot_base_frame>
     ```
   - **Note:** For Cartographer to work, you must set `odom_tf` to `false`, and configure the `odom` topic as needed.

### 3. How to Perform Simulation Using Cartographer

1. **If you want to perform mapping using Cartographer:**
   ```bash
   $ ros2 launch f1_robot_model cartographer.launch.py
   ```
   - You can tune Cartographer parameters by modifying the `.lua` file located in `~/src/f1_robot_model/config`.

2. **If you want to perform localization with Cartographer and publish the odom topic and odom TF:**
   ```bash
   $ ros2 launch f1_robot_model localization.launch.py
   ```
   - Similarly, you can tune the parameters by modifying the corresponding `.lua` file.
