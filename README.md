** If you would like to read the English version of the document, please read the document containing English **
## 전체 요약
---
본 패키지는 F1TENTH를 위한 Gazebo classic 베이스의 시뮬레이터를 개발하는 것을 목적을 개발되었습니다. 본 패키지는 여러 패키지를 포함하고 있는 메타-패키지기 때문에 사용이 상당히 복잡하므로, 여러 설명서를 포함하고 있습니다. 따라서 설명서를 주의 깊게 읽고 진행하는 것을 추천드립니다.

만약, 사용에 문제점 혹은 의문점이 있다면
** alpha12334@naver.com ** 혹은 ** suberkut76@gmail.com ** 으로 연락 부탁드립니다.

본 패키지는 다음과 같은 구성으로 되어 있습니다.
- ** cartographer ** : C++로 작성된 SLAM 패키지
- ** cartographer_ros ** : cartographer의 ROS2 Wrapper
- ** f1_robot_model ** : F1TENTH 모델링 및 각종 launch, 설정 파일이 담긴 패키지
- ** velodyne_simulator ** : 3D LiDAR 시뮬레이션을 위한 의존성 패키지

또한, 이해를 돕기 위해서 다음과 같은 설명서를 보유하고 있습니다.
- README.md (Korean and English)
- USERGUID.md (Korean and English)
- CHANGELOG.md (Korean and English)

본 패키지를 개발하는데 있어서, 매우 큰 도움이 되었던 다음 오픈소스 개발진들에게 감사하다는 말씀을 드리고 싶습니다.
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [cartographer_ros](https://github.com/ros2/cartographer_ros)
- [f1tenth_gtc_tutorial](https://github.com/linklab-uva/f1tenth_gtc_tutorial)
- [f1_robot_model](https://github.com/armando-genis/f1_robot_model)
- [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator.git/src)
