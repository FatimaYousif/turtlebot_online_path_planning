# Turtlebot Online Path Planning

In this Hands-on Planning (HOP) lab attached with its package deliverable, the first online path planning pipeline for Turtlebot3 was further developed. Having included the following classes as StateValidityChecker, Planner (RRT with smoothing) , and compute_path(), and move_to_point() functions alongside the OnlinePlanner class for online path planning and controlling a robot using ROS with Key functionalities including: Subscribing to various topics, implementing methods, providing velocity controller, implementing robot recovery (from obstacle avoidance) behaviour case to publishing markers for visualization in RViz. 
<br><br>

## Lab contributors & student ID:

| Name                    | ID       |
| -------------------     | -------- |
| Fatima Yousif Rustamani | u1992375 |
| Lisa Paul Magoti        | u1992463 |

## HOW TO RUN:

### 1. Launch Gazebo and RViz

```bash
roslaunch turtlebot_online_path_planning gazebo.launch
```

### 2. New terminal

```bash
rosrun turtlebot_online_path_planning turtlebot_online_path_planning_node.py
```

## Special (Extra) Functionality:

The following are the added generic functions besides the RRT or Planner class´s auxiliary functions.

### __in_map__(self, loc)

The purpose of this general in-line method in the StateValidityChecker class is to check whether a given location (specified as a list of indices [x, y]) is within the boundaries of the map -  where called and necessary. It returns True if the location is within the map boundaries and False otherwise.

Used in: __position_to_map__, __check_vicinity__, and __is_valid__ functions

### __check_vicinity__(self, cell, distance, checking_path=False)

Another general method within the StateValidityChecker class, this function checks the vicinity of a given cell within a certain distance. It iterates through the cells around the specified cell within the given distance and checks if any of them are obstacles. If checking_path is True, it considers a reduced distance to check for obstacles, for more frequent checks during path planning. It returns True if the vicinity is free from obstacles and False otherwise. 

Used in: __is_valid__ function

### __recover__(self)

The recover method in the OnlinePlanner class handles recovery behavior when the robot is stuck in an obstacle. It sets the robot to move backward for a predefined recovery duration to try to get it out of the stuck situation.

These methods are crucial for ensuring safe navigation of the robot within its environment, detecting obstacles, and handling situations where the robot gets stuck. They form part of a larger set of functionalities for online path planning and robot control.

#### RViz Visualization of robot:

![Video of the robot planning the path](./docs/img/costmap_exploration.png)


