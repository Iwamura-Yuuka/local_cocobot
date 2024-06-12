# local_cocobot

Local path planner considering costs based on travel predictions

<p align="center">
  <img src="https://github.com/Iwamura-Yuuka/local_cocobot/blob/main/images/coco50_demo.gif" height="500px"/>
</p>

## Environment
- Ubuntu 20.04
- ROS Noetic

## Dependencies
- [sq2_ccv_description](https://github.com/amslabtech/sq2_ccv_description)
- [ccv_pure_pursuit_steering](https://github.com/amslabtech/ccv_pure_pursuit_steering)
- [steering_path_planner](https://github.com/amslabtech/steering_path_planner)
- [SPACiSS](https://github.com/Iwamura-Yuuka/SPACiSS)

## Install and Build
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/Iwamura-Yuuka/local_cocobot.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths .  # Install dependencies
catkin build
```

## How to use
### Proposed Method + Differential Drive Robot with Steering
**50 pedestrians**
```
roslaunch ccv_experiment coco_all50.launch
```
**25 pedestrians**
```
roslaunch ccv_experiment coco_all25.launch
```
**1 pedestrian**
```
roslaunch ccv_experiment coco_all1.launch
```
### Proposed Method + Differential Drive Robot(without Steering)
**50 pedestrians**
```
roslaunch ccv_experiment no_steer50.launch
```
**25 pedestrians**
```
roslaunch ccv_experiment no_steer25.launch
```
**1 pedestrian**
```
roslaunch ccv_experiment no_steer1.launch
```