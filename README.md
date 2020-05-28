# Pick and Place with Franka Robot using GPD

## ROS Wrapper for GPD Grasping Algorithm

A ROS wrapper around the [GPD](https://github.com/SnehalD14/gpd) package for detecting 6-DOF grasp poses for a
2-finger robot hand ( a parallel jaw gripper) in 3D point clouds.

## Installation

The following instructions have been tested on **Ubuntu 16.04**.

1. Install GPD. You can follow [these instructions](https://github.com/SnehalD14/gpd).

2. Clone this repository into the `src` folder of your catkin workspace:

   ```
   cd <location_of_your_workspace>/src
   git clone https://github.com/SnehalD14/gpd_ros
   ```

3. Build your catkin workspace:

   ```
   cd <location_of_your_workspace>
   catkin_make
   ```
4. For setting up the Franka simulation environment with Gazebo and Moveit, install [panda_simulation](https://github.com/SnehalD14/panda_simualtion).


## Instructions 

1. Launch the simulation environment from `panda_simualation`

   ```
   roslaunch panda_simulation simulation.launch 
   ```

2. Launch GPD planner service

   The following command will launch a ROS node
   that waits for point clouds on the ROS topic `/r200/camera/depth_registered/points`. Once a point
   cloud is received, the node will search the cloud for grasps.

   ```
   roslaunch gpd_ros gpd_planner_service.launch
   ```
   Make sure to close the visualization window for the publisher to recieve the grasps. 

3. Run the GPD execution code

   We subscribe to the grasps planned using GPD and obtain position and orientation. This code makes the robot move the 
   object, grasps it and move away from the table with the grasped object. 
          

   ```
   python scripts/gpd_execution.py
   ```
## References

Andreas ten Pas, Marcus Gualtieri, Kate Saenko, and Robert Platt. [**Grasp
Pose Detection in Point Clouds**](http://arxiv.org/abs/1706.09911). The
International Journal of Robotics Research, Vol 36, Issue 13-14, pp. 1455-1473.
October 2017.

