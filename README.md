# coExplore repository for multi-robot exploration
---
This repository contains research code for the development of our novel multi-robot exploration algorithm built with ROS1 called coExplore. 

### System Prerequisites
This project was built with   
- ROS Noetic on Ubuntu 20.04.

### Install
</br>
This project is an extension of an existing repository for robotino robots.
To install these dependencies, first run the install_script.bash from a suitable catkin directory. This completely installs all the needed packages to run a minimal working version of the project.
</br>
</br>

### Configuration

Sourcing the config_script will give a printout of the options available
as well as the currently active setup.

```
-------------------------------------------------------------
coExplore configuration
-------------------------------------------------------------
tme_ROBOT_ENV: [marty, v2_maze, map_3]
tme_start_num: [2 - 5]
tme_expl_method: [nearest, minPos, nextFrontier, coExplore, co122]
tme_stop_time: number in seconds the simluation should run
-------------------------------------------------------------

```
eg. change the map with ``export tmb_ROBOT_ENV=map_3``

# Module Summary

### Perception

The perception module uses sensor data from the camera, laser scanner and odometry of the non-blind robots to predict the state of the blind robot. This includes a trained deep learning module for object detection, the interpretation of detected bounding boxes into the detecting robot's local coordinate frame, and then synthesizing this with the recent history of the blind robot to have an estimate of its location and bearing in the global frame.


### SLAM
For Simultaneous Localization and Mapping (SLAM) we use the slam_toolbox package. Configuration files are in the config folder in the tme_startup package. With map merging we are able to compute an overall map which gets explored by both robots where we can locate the blind robot in. In the accompanying videos you can see both robots exploring on their respective maps in the top and the resulting merge in the bottom.


### Path Planning
Path planning works in two different phases. First we explore the environment by finding unknown space and creating frontiers, this is done by the explore-lite package. We then publish a goal while trying to explore the biggest frontiers. The path is computed by the move_base package by computing a global costmap on the robot maps and then utilizing the laser sensors to perceive the immediate environment and adjust to dynamic obstacles through the local costmap.

After perceiving the goal, the blind robot and being able to compute a path. We switch to the guiding routine which disables the exploration and allows the robots to move to the blind robot and guide it to the goal. The path planning works similarly like before but the goal publishing nodes change.

---
# Detailed Overview

## robot_state_publisher
This [package](http://wiki.ros.org/robot_state_publisher) allows you to publish the state of a robot to tf2. Once the state gets published, it is available to all components in the system that also use tf2. The package takes the joint angles of the robot as input and publishes the 3D poses of the robot links, using a kinematic tree model of the robot. Since in ROS noetic tf is deprecated in favor of tf2, the concept of a multi robots system using tf_prefix was not possible. Tf_prefix is designed for operating multiple similar robots in the same environment. The tf_prefix parameter allows two robots to have base_link frames, for they will become /robot1/base_link and /robot2/base_link if all nodes for robot1 are run in robot1 as their tf_prefix parameter. Therefore, we had to manually modify the **robot_state_publisher** package to be able to run multiple robots in our simulation.

## Perception


#### __Bounding Box Interpreter__
This node makes sense of the information received from bounding boxes.
Notably, it predicts the distance, and rough bearing estimate of the detected object.

The laser scans in a wide area around the robot.
Those that overlap with the camera feed are synthesized and scaled [0,1] corresponding to far left and far right of the camera feed.

The location of the bounding box within the camera frame is likewise scaled.
The distance of the object is thus predicted from taking the scans which relate
to the bounding box.

#### __Object Detector__

This node processes the information from the perception module and makes predictions
about the location of detected objects. However, to keep the perception pipeline as a
detachable module it also provides an opportunity to simulate (mock) the camera module. Regardless of which option we choose, we created an interface which allows us to swap options interchangeably without changing anything in the implementation. How the different information is gathered is described in the following table.

Mock | Estimator
---|---
Taking properties from the camera itself, as well as the known robot positions it is estimated whether the camera "would" otherwise be able to detect the object. In this case, we publish the information which that robot would be expected to see. ie, if it rotates out of the line of sight of the target, publishing stops. | Using the information from the bounding box, and the known information about the robot itself, form an estimation of the location of the spotted target. Using this position estimate, as well as velocity input, to determine bearing.

## SLAM
We use [slam_toolbox](http://wiki.ros.org/slam_toolbox) with a largely base setup. The configuration is specified in 

## map_merge
To merge the maps acquired by the SLAM we use the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) package. This allows us to merge maps where the robot start positions are known. In theory the algorithm is also able to compute maps without knowing the start positions of the robot. This did not work in practice but we could overlay with known starting positions anyway. For known start positions the maps get overlayed. This means that deviations in SLAM lead to large deviations in the computed merged map. So a good SLAM is crucial for this to work properly.

## move_base
The [move_base](http://wiki.ros.org/move_base) ROS Node is a major component of the [Navigation Stack](http://wiki.ros.org/navigation).

The move_base package provides an implementation of an action (from action_lib) that, given a goal in the world, will attempt to reach it with a mobile base. The **move_base** node links together a **global and local planner** to accomplish its global navigation task. It supports any global planner adhering to the BaseGlobalPlanner interface specified in the nav_core package and any local planner adhering to the BaseLocalPlanner interface specified in the same package. The move_base node also maintains **two costmaps**, one for the global planner, and one for a local planner

# API

**Bounding box interpreter**
```yaml
topic: /tmb_perception/bounding_box_interpretation

Bounding_Box_Interpretation
string: detected_by
string: object_detected
float32: distance
float32: camera_center_position
```
**Pose Estimator**
```yaml
topic: /{robot_name}/tmb_computed_pose

Computed_Pose
float32: x
float32: y
float32: yaw
```
**Object Detector**
```yaml
topic: /tmb_perception/object_sighted

Object_Sighted
string: detected_by
string: object_detected
string: object_to_the_left_or_right
float32: distance
float32: incidence
geometry_msgs/Point: object_position_estimate
```

---
# Possible further development

- Multi robot exploration algorithm to explore the map in a more distributed fashion
- Update merged map directly after robots localize themselves in it
- Introduce possibility to start from unknown positions
- Improve durability of system
- Introduce automatic error handling for more sustainable performance in long scenarios
- Use filtering techniques to improve control inaccuracies in the estimated pose calculations
- Treat guiding as a path search problem using only the initial pose 