## 기본적인 안내
- `f1_robot_model` : Gazebo classic 시뮬레이션을 위한 `URDF, SDF` 파일, cartographer 시뮬레이션을 위한 `lua, pbstream` 파일 등을 제공
- `cartographer_ros` : Mapping과 Localization을 위한 `SLAM` 기능 제공

### 1. 설치 방법
```bash
$ git clone
$ cd
$ colcon build --symlink-install
$ source install/setup.bash
# source install/local_setup.bash
```

### 2. gazebo classic simulation 사용 방법
1. 순수하게 로봇만 `gazebo`에 `spawn`하고 싶다면
```bash
$ ros2 launch f1_robot_model robot_spawn.launch.py
```

2. 내가 작성한 `world 파일(.sdf)`까지 `spawn`하고 싶다면
```bash
$ ros2 launch f1_robot_model display.launch.py
```
- `~/src/f1_robot_model/world` 파일 내부, 그리고 `display.launch.py` 파일의 `world_path` 부분을 수정해서 커스텀 `world` 파일을 upload 및 spawn 할 수 있음

3. URDF 파일로부터 odom 토픽과 odom 프레임이 자동으로 나오게 하려면
- `~/src/f1_robot_model/urdf/racecar.gazebo` 내부 코드 수정
```xml
      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_distance>true</publish_distance>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
```
- `cartographer` 동작을 위해서는 `odom_tf`를 `false`로 해야하며, `odom` 토픽은 필요에 따라서 정해쓸 것


### 3. cartographer를 통해 simluation 하는 방법
1. cartographer를 이용해서 Mapping을 수행하고 싶다면
```bash
$ ros2 launch f1_robot_model cartographer.launch.py
```
- `~/src/f1_robot_model/config` 파일 내부의 `.lua` 파일을 수정해서 `cartographer` 파라미터 튜닝을 수행할 수 있음

2. cartographer를 이용해서 localization을 수행하고, odom 토픽과 odom TF를 pub하고 싶다면
```bash
$ ros2 launch f1_robot_model localization.launch.py
```
- 3번과 마찬가지로 `.lua` 파일을 수정해서 튜닝할 수 있음
