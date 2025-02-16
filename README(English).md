**If you would like to read the Korean version of the document, please read the document containing Korean.**

## Overall Summary
---
This package was developed with the aim of creating a Gazebo Classic-based simulator for F1TENTH. Since this package is a meta-package that includes several sub-packages, its usage is quite complex, and multiple documents are provided. It is therefore recommended that you carefully read the documentation before proceeding.

If you encounter any issues or have any questions regarding its usage, please contact **alpha12334@naver.com** or **suberkut76@gmail.com**.

This package is composed of the following components:
- **cartographer**: A SLAM package written in C++.
- **cartographer_ros**: A ROS2 wrapper for Cartographer.
- **f1_robot_model**: Contains the F1TENTH robot model along with various launch and configuration files.
- **velodyne_simulator**: A dependency package for 3D LiDAR simulation.

In addition, to assist with understanding, the following documentation is provided:
- README.md (Korean and English)
- USERGUID.md (Korean and English)
- CHANGELOG.md (Korean and English)

We would like to express our gratitude to the following open-source developers, whose contributions were immensely helpful in the development of this package:
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [cartographer_ros](https://github.com/ros2/cartographer_ros)
- [f1tenth_gtc_tutorial](https://github.com/linklab-uva/f1tenth_gtc_tutorial)
- [f1_robot_model](https://github.com/armando-genis/f1_robot_model)
- [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator.git/src)
