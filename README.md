# ME495 Embedded Systems Homework 4

Author: [Zhengxiao Han (韩 政霄)](https://0nhc.github.io)

`nubot_nav`: This package contains navigation application for nubot.

## Dependencies
I am using `cartographer_ros` instead of `slam_toolbox` because the mapping performance of `slam_toolbox` is poor in the given scene. This issue may stem from inaccurate odometry provided by Gazebo, likely due to incorrect parameters in the diff_drive plugin.

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

