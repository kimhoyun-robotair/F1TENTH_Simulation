## Changelog for cartographer_ros

### As of February 10, 2025
- **Modified CMakeLists.txt file**: Added content related to the `/odom` topic.
  ```cmake
  add_executable(trajectory_to_odom src/trajectory_to_odom.cpp)
  ament_target_dependencies(trajectory_to_odom rclcpp visualization_msgs geometry_msgs std_msgs nav_msgs tf2 tf2_geometry_msgs)  # Ensure nav_msgs is listed
  install(TARGETS trajectory_to_odom DESTINATION lib/${PROJECT_NAME})
  ```
- **Modified package.xml file**: Added dependency related to the `/odom` topic.
  ```xml
    <depend>tf2_geometry_msgs</depend>
  ```
- **Modified `node_constants.h` and `node_options.h` in `/cartographer_ros/include/cartographer_ros`**
  - *node_constants.h*: Modified the IMU topic name.
    ```C++
    constexpr char kImuTopic[] = "imu/data";
    ```
  - *node_options.h*: Modified the code related to Pose output.
    ```C++
    bool publish_tracked_pose = true;
    ```
- **Added `trajectory_to_odom.cpp` code** to the `/src` directory.
- Added Localization and Odometry output using `Cartographer`.

### As of 2025.02.16
- **Added launch and configuration files for AMCL**: Enables Localization using AMCL.
- **Integrated `teleop_twist_joy` and MXswitch controllers**: Enables efficient mapping using a joystick during mapping.
- **Added 3D LiDAR simulation feature**: Enables 3D LiDAR simulation using Velodyne LiDAR.

### As of February 17, 2025
- **GPS Plugin Added**: GPS data and topics can now be utilized within the simulation.
