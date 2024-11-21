# gen3_moveit
for visionless Peg-In-Hole with Spiral Trajectory & Implementing objects at Gazebo
## Environment Used
* Ubuntu 18.04
* ROS Melodic
* Kinova Gen3
* Robotiq 2f-85 Gripper

[gen3_lite_examples](https://github.com/AIRLABkhu/gen3_lite_examples)
[ROS Kortex]((https://github.com/Kinovarobotics/ros_kortex))

## Usage
Gazebo에 Robot arm과 world 불러오기.

Workspace setup은 [MoveIt Getting Started](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) 참조
```
roslaunch gen3_moveit gen3_gazebo.launch
```
manipulation 코드 실행
```
roslaunch gen3_moveit gen3_pick_place.launch
```
