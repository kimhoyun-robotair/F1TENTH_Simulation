## 변화 내역 정리

### 2025.02.10 기준
- **CMakeLists.txt 파일 수정** : `/odom` 토픽 관련 내용 추가
```
add_executable(trajectory_to_odom src/trajectory_to_odom.cpp)
ament_target_dependencies(trajectory_to_odom rclcpp visualization_msgs geometry_msgs std_msgs nav_msgs tf2 tf2_geometry_msgs)  # Ensure nav_msgs is listed
install(TARGETS trajectory_to_odom DESTINATION lib/${PROJECT_NAME})
```
- **package.xml 파일 수정** : `/odom` 토픽 관련 의존성 추가
```xml
  <depend>tf2_geometry_msgs</depend>
```
- **`/cartographer_ros/include/cartographer_ros`의 `node_constants.h`와 `node_options.h` 수정**
 ! node_constants.h : IMU 토픽 이름 수정
    ```C++
    constexpr char kImuTopic[] = "imu/data";
    ```
 ! node_options.h : Pose 출력 관련 코드 수정
    ```C++
    bool publish_tracked_pose = true;
    ```

- **`/src`에 `trajectory_to_odom.cpp` 코드 추가**
- `Cartographer`를 활용한 Localization 및 Odometry 정보 출력 추가

### 2025.02.16 기준
- **AMCL 관련 launch 파일 및 config 파일 추가** : AMCL을 활용한 Localization 가능
- **teleop_twist_joy 및 MXswitch 컨트롤러 통합** : 매핑시 조이스틱을 활용한 효율적 매핑 가능
- **3D LiDAR 시뮬레이션 기능 추가** : velodyne LiDAR를 활용한 3D LiDAR 시뮬레이션 가능

### 2025.02.17 기준
- **GPS 플러그인 추가** : GPS 데이터 및 토픽을 시뮬레이션 안에서 활용 가능
