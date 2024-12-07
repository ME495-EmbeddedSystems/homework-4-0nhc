# ME495 Embedded Systems Homework 4

Author: [Zhengxiao Han (韩 政霄)](https://0nhc.github.io)

`nubot_nav`: This package contains navigation application for nubot.

## Dependencies
I am using `cartographer_ros` instead of `slam_toolbox` because the mapping performance of `slam_toolbox` is poor in the given scene. The direct issue appears to be the inaccuracy of odometry, which might be caused by missing friction configurations in Gazebo. I recommend checking the friction parameters in [nubot](https://github.com/m-elwin/nubot).

To install the necessary dependencies, simply run the following commands:
```sh
cd <your_ws>
rosdep install --from-paths src --ignore-src -r
```

## Quickstart
* **Manual Exploration**
  ```sh
  ros2 launch nubot_nav manual_explore.launch.py
  ```
  https://github.com/user-attachments/assets/6f293f63-f371-44b5-8c82-9ef3978172e2

* **Automatic Exploration (Wall Following)**
  
  I am using a wall-following algorithm to automatically explore the entire scene. The robot uses sensor data (LaserScan) to detect obstacles (walls), then moves along the walls while maintaining a safe distance. It smoothly handles turns and avoids obstacles, ensuring that all reachable areas in the scene are explored. See details in [explore.py](https://github.com/ME495-EmbeddedSystems/homework-4-0nhc/blob/main/nubot_nav/explore.py).
  ```sh
  ros2 launch nubot_nav explore.launch.py
  ```
  https://github.com/user-attachments/assets/321767ee-9e54-45a9-83a9-d13ca1f98eb5

## Improvements
* **Friction Parameters**:
  
  As mentioned earlier, the [nubot](https://github.com/m-elwin/nubot) package lacks friction parameters. These should be added to the wheels, caster wheels, and ground to improve simulation accuracy.
* **Extended Kalman Filter (EKF)**: 
  
  I have tried adding IMU plugins in [my forked nubot repository](https://github.com/0nhc/nubot). With the IMU sensor, we could use `robot_localization` to fuse odometry and IMU data. However, as I have tried, the performance is still poor.

* **Ground Truth Odometry**:
  
  As I have applied in [homework3](https://github.com/ME495-EmbeddedSystems/homework-3-0nhc/blob/main/diff_drive/urdf/ddrive.gazebo.xacro), Use odometry_publisher plugin to publish ground truth odometry.