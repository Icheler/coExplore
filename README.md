# Tu Many Robots @TUM
---


```
Two capable robots are spawned into an unknown world.  
They must locate the exit, as well as a blind robot, i.e. without visual sensors.    
They must guide the blind robot to the exit.   
```

<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%"; class="center">
  <tr>
    <td style="width: 50%;"> <img src="Demo/topleft.gif" width='400'></td>
    <td style="width: 50%;"> <img src="Demo/topright.gif" width='400'></td>
  </tr>
  <tr>
    <td style="width: 50%;"> <img src="Demo/bottomleft.gif" width='400'></td>
    <td style="width: 50%;"> <img src="Demo/bottomright.gif" width='400'></td>
  </tr>
</table>

---
### System Prerequisites
This project was built with   
- ROS Noetic on Ubuntu 20.04.
- NVIDIA GPU **

** This computer vision pipeline requires a Nvidia GPU. However, a mock of the pipeline is made so that the project can be run without it. Testing was done with an NVIDIA 2060 Super with a AMD Ryzen 7 3700X. As the simulation runs 3 robots and 2 visual pipelines the visualization of results is computationally expensive and affects performance while running the complete system.

---

### Install
</br>
This project is an extension of an existing repository for robotino robots.
To install these dependencies, first run the install_script.bash from a suitable catkin directory. This completely installs all the needed packages to run a minimal working version of the project.
</br>
</br>

The vision module additionally requires following the README within tme_perception package. By default, a mock of the pipeline is used which does not need a GPU. Attempting to build the vision module without installing the dependencies will fail. Hence, **blacklist** the darknet_ros package

---
### Build

```
cd <your catkin workspace>

catkin config --blacklist darknet_ros
catkin build
source devel/setup.bash
source src/TU_Many_Bots/config_script.bash
```


Sourcing the config_script will give a printout of the options available
as well as the currently active setup.

```
-------------------------------------------------------------
TU Many Bots configuration
-------------------------------------------------------------
tme_ROBOT_ENV: [simple_corridor, maze, maze_clutter, maze_clutter_limited]
tme_start_num: [true, false]
tme_publish_perception_logs: [true, false]
-------------------------------------------------------------
selected world is: simple_corridor
starting both robots: True
publishing perception logs: False
with predicting yaw: False
-------------------------------------------------------------

```
eg. change the map with ``export tme_ROBOT_ENV=simple_corridor``

---
### Run


The simulation can be started with:
```
roslaunch tme_startup complete_launch.launch
```

Regardless of whether the computer vision pipeline is enabled for this computation, the module can always be run independently to see its performance.

```
roslaunch darknet_ros darknet_ros.launch
```

---

# Module Summary

![nodes diagram](Demo/nodes_no_tf.png)

### Perception

The perception module uses sensor data from the camera, laser scanner and odometry of the non-blind robots to predict the state of the blind robot. This includes a trained deep learning module for object detection, the interpretation of detected bounding boxes into the detecting robot's local coordinate frame, and then synthesizing this with the recent history of the blind robot to have an estimate of its location and bearing in the global frame.


### SLAM
For Simultaneous Localization and Mapping (SLAM) we use the standard gmapping package. Configuration files are in the config folder in the tme_startup package. Gmapping was chosen over the slam-toolbox online async algorithm after evaluating both algorithms in testing. Even after including more cluttering to the maps, the toolbox still had trouble providing twist free maps and it was also getting lost during loop closures. Gmapping has the distinct advantage that our map merge algorithm works best with maps with fixed sizes which are provided by gmapping. With map merging we are able to compute an overall map which gets explored by both robots where we can locate the blind robot in. In the accompanying videos you can see both robots exploring on their respective maps in the top and the resulting merge in the bottom.

<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%" class="center">
  <tr>
    <td style="width: 50%;"> <img src="Demo/topleft.gif" width='400'></td>
    <td style="width: 50%;"> <img src="Demo/topright.gif" width='400'></td>
  </tr>
  <tr>
    <td style="width: 50%;"> <img src="Demo/bottomleft.gif" width='400'></td>
  </tr>
</table>


### Path Planning
Path planning works in two different phases. First we explore the environment by finding unknown space and creating frontiers, this is done by the explore-lite package. We then publish a goal while trying to explore the biggest frontiers. The path is computed by the move_base package by computing a global costmap on the robot maps and then utilizing the laser sensors to perceive the immediate environment and adjust to dynamic obstacles through the local costmap.

After perceiving the goal, the blind robot and being able to compute a path. We switch to the guiding routine which disables the exploration and allows the robots to move to the blind robot and guide it to the goal. The path planning works similarly like before but the goal publishing nodes change.


### Guiding Routine
The guiding module, which serves the purpose of leading the blind robot to the goal position, actively publishes the estimated poses of all the robots, then through transformations the control outputs (consisting of velocity profiles) are applied to the robots creating a single file chain to navigate the blind robot.


![Guiding routine](Demo/bottomright.gif)

---
# Detailed Overview

## robot_state_publisher
This [package](http://wiki.ros.org/robot_state_publisher) allows you to publish the state of a robot to tf2. Once the state gets published, it is available to all components in the system that also use tf2. The package takes the joint angles of the robot as input and publishes the 3D poses of the robot links, using a kinematic tree model of the robot. Since in ROS noetic tf is deprecated in favor of tf2, the concept of a multi robots system using tf_prefix was not possible. Tf_prefix is designed for operating multiple similar robots in the same environment. The tf_prefix parameter allows two robots to have base_link frames, for they will become /robot1/base_link and /robot2/base_link if all nodes for robot1 are run in robot1 as their tf_prefix parameter. Therefore, we had to manually modify the **robot_state_publisher** package to be able to run multiple robots in our simulation.

## Perception


<img src="Demo/detection.png">

> "*What the guiding robots tells us about the blind robot*"


It is a **pipeline** consisting of:
- Detecting the blind robot with a camera -> bounding boxes
- Synthesizing the bounding box + laser sensor data -> relative position
- Combining the relative position + known position of detecting robot -> absolute position
- Using this position estimate + as well as velocity input -> bearing


Within the simulation, the bearing and absolute location of the blind robot is known.
Not only does this mean that we can bypass the entire pipeline, we have also isolated each step of the pipeline.

This can be done by creating a **mock** of each step.

The bounding box is mocked by assuming a span of vision from the camera sensor
and only publishing the detected position if we could realistically have expected the robot to have seen the blind robot.

Mocking the yaw is done by passing through the known yaw, instead of the computed yaw.

Whether any steps of the pipeline are mocked during execution can be toggled by setting
any combination of the following environment variables to true


> tme_with_computed_yaw  
> tme_with_computer_vision *

by isolating each step, and having the expected value available, it is thus able
to independently test each step, and also to compare the accuracy of each step.

`*`note: although each step of the tme_with_computer_vision pipeline was tested in isolated, the project team did not have hardware capable of running the full vision pipeline, so the option to run the full vision pipeline cannot be verified and is therefore stashed on a separate branch.


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


#### __Pose Estimator__
The pose estimator acts as an interface for the following and guiding routines.
Robots with sufficient sensor input are able to determine their own pose and this is accordingly passed along unchanged. The blind robot instead needs to have its pose estimated.

Information used:
  - Global position estimates received from the guiding robot
  - Input velocities provided to the blind robot

A series of global position estimates are combined to form a short term computed estimate of the robot translation. This, when compared with the linear input velocity of the local frame generates a bearing estimate. For example, an attempt to go forward according to the velocity which results in backward translation indicates a downward bearing. However, angular rotation can occur in the absence of translation. Thus angular rotation as measured by integrating the angular velocity is independently added to the bearing estimate.

Application|Description
---|---
computed_translation|unit vector of movement in global frame
computed_velocity|unit vector of linear velocity in robot local frame
displacement|angular rotation in radians


## SLAM
We use [gmapping](http://wiki.ros.org/gmapping) with a largely base setup. We changed the parameters so the map gets updated at a rate of 1Hz. Space over 5 meters away gets classified as unknown space which allows to compute frontiers in exploration. Configuration is specified in the tme_startup package under config.

## map_merge
To merge the maps acquired by the SLAM we use the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) package. This allows us to merge maps where the robot start positions are known. In theory the algorithm is also able to compute maps without knowing the start positions of the robot. This did not work in practice but we could overlay with known starting positions anyway. For known start positions the maps get overlayed. This means that deviations in SLAM lead to large deviations in the computed merged map. So a good SLAM is crucial for this to work properly.

## explore_lite
We use the explore_lite package provided by http://wiki.ros.org/explore_lite. This is developed by the same developer as the multirobot_map_merge package we explained previously. The only adjustments to the algorithm are a change in topics and increasing the timeout period so frontiers get only classified as unreachable after a longer period of time. The algorithm tracks unknown space in the provided map to compute frontiers. Then by finding the biggest frontier, a goal is published on the specified topic and then we use move_base to travel to that frontier. Map updates lead to new frontiers, which will then impact the computed goal so the biggest frontiers get explored first in a greedy approach. Drawbacks of this approach can be found when looking at time needed to explore a complete space, since only the biggest frontiers get explored, newly found frontiers in the vicinity of the robot get explored at a later time.

## move_base
The [move_base](http://wiki.ros.org/move_base) ROS Node is a major component of the [Navigation Stack](http://wiki.ros.org/navigation).

The move_base package provides an implementation of an action (from action_lib) that, given a goal in the world, will attempt to reach it with a mobile base. The **move_base** node links together a **global and local planner** to accomplish its global navigation task. It supports any global planner adhering to the BaseGlobalPlanner interface specified in the nav_core package and any local planner adhering to the BaseLocalPlanner interface specified in the same package. The move_base node also maintains **two costmaps**, one for the global planner, and one for a local planner

## Guiding
Once the problem statement is solved (i.e the mapping robots found both the goal position and the blind robot and a path exists between them) the problem switches from the mapping and exploration phase to the guiding phase, which is implemented via the guiding routine found in the **tme_follow** package.

The Guiding routine receives the blind robot's pose estimated using the **pose_estimator** node from the peception module, together with the estimated poses of the guiding robots. The routine utilizes **TF** to calculate the control signals to obtain the required velocity profiles for the robots to move in a single file-like chain to the goal getting the blind robot to the required position. The routine also uses several services for action distinction and ensuring a collision free interaction while proceeding to the goal.

## Following
The routine consists of **two nodes**, the broadcaster plays the role of reading the position estimates of the robots including the blind_robot [via odom or pose_estimator] and re-broadcasts them as tf frames relative to the Global Map Topic: /map
The launch file tme_follow.launch assigns the frames for all the robots in the simulation, visit to adjust accordingly. The broadcaste subscribes to the respective robots namespace /odom topic and message type Odometry. The second node responsible for the grunt of the work in the follow
subroutine. The node listens to the transforms broadcasted by its  sister node and uses the data, when the conditions in the exploration phase are met, to guide the blind_robot to the goal position.

The node utilizes three Trigger services to give the individual robots  their roles in the follow routine, the transform listener is used afterwards to calculate and publish the required velocity profiles. The services provide the required chain of events and collision avoidance in case the robots follow each other too closely.

# API

**Bounding box interpreter**
```yaml
topic: /tme_perception/bounding_box_interpretation

Bounding_Box_Interpretation
string: detected_by
string: object_detected
float32: distance
float32: camera_center_position
```
**Pose Estimator**
```yaml
topic: /{robot_name}/tme_computed_pose

Computed_Pose
float32: x
float32: y
float32: yaw
```
**Object Detector**
```yaml
topic: /tme_perception/object_sighted

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
