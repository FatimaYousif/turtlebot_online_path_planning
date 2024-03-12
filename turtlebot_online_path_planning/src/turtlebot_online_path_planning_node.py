#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from utils_lib.online_planning import StateValidityChecker, move_to_point, compute_path

class OnlinePlanner:

    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, bounds, distance_threshold):

        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(distance_threshold)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.bounds = bounds                                        

        # CONTROLLER PARAMETERS
        # Proportional linear velocity controller gain
        self.Kv = 2
        # Proportional angular velocity controller gain                   
        self.Kw = 3
        # Maximum linear velocity control action                   
        self.v_max = 0.15
        # Maximum angular velocity control action               
        self.w_max = 0.3       

        self.max_retries = 10   # Maximum number of retries
        self.max_planning_time = 10 # Initial planning time limit (s)
        self.reserved_time = 10   #  planning time to increse after fail (s)
      
        # Recovery Behavior Parmeters 
        self.recovery_time = rospy.Duration(5)
        self.recovery_vel = -0.1

        #--------------------ROS NODE-------------------------
        # PUBLISHERS
        # Publisher for sending velocity commands to the robot
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1) # TODO: publisher to cmd_vel_topic
        # Publisher for visualizing the path to with rviz
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        
        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap) # TODO: subscriber to gridmap_topic from Octomap Server  
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom) # TODO: subscriber to odom_topic  
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal) # TODO: subscriber to /move_base_simple/goal published by rviz    
        

        # TIMERS
        # Timer for velocity controller
        rospy.Timer(rospy.Duration(0.1), self.controller)
    
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])

        # TODO: Store current position (x, y, yaw) as a np.array in self.current_pose var.
        self.current_pose =  np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]) 
        # ----- WE CAN ADD VELOCITY too = FOR CLOSED LOOP -----------
    
    # Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method
    def get_goal(self, goal):
        if self.svc.there_is_map:
            # TODO: Store goal (x,y) as a numpy aray in self.goal var and print it 
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            print("Received new goal:", self.goal)
            
            if self.svc.is_valid(self.goal):
                # Plan a new path to self.goal
                self.plan()

            else:
                print("Goal is not valid")
        
    # Map callback: Gets the latest occupancy map published by Octomap server and update 
    # the state validity checker
    def get_gridmap(self, gridmap):
      
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)

            # If the robot is following a path, check if it is still valid
            if self.path is not None and len(self.path) > 0:

                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [self.current_pose[0:2]] + self.path

                # TODO: check total_path validity. If total_path is not valid replan
                if not self.svc.check_path(total_path):
                    rospy.logerr("Error: Current path is invalid. Replanning...")
                    self.path = []
                    self.plan()
                else:
                    rospy.logwarn("Warning: Current path is still valid. Continuing on the same path.")


    # Solve plan from current position to self.goal. 
    def plan(self):
        # Invalidate previous plan if available
        self.path = []

        # Check if robot stuck in the obstacle 
        while self.is_robot_in_obstacle():
            self.recover()
            self.path = compute_path(start_p= self.current_pose[0:2],goal_p=self.goal[0:2],
                                 state_validity_checker=self.svc, bounds= self.bounds,max_time=self.max_planning_time) 
        # TODO: plan a path from self.current_pose to self.goal
        self.path = compute_path(start_p= self.current_pose[0:2],goal_p=self.goal[0:2],
                                 state_validity_checker=self.svc, bounds= self.bounds,max_time=self.max_planning_time)
        
        # TODO: If planning fails, consider increasing the planning time, retry the planning a few times, etc.
        if len(self.path) == 0:
            for i in range(self.max_retries):
                self.path = compute_path(start_p= self.current_pose[0:2],goal_p=self.goal[0:2],
                                    state_validity_checker=self.svc, bounds= self.bounds, max_time=self.max_planning_time+ self.reserved_time) 
                if self.path:
                    break
        
        if len(self.path) == 0:
            print("Path not found!")
        else:
            print("Path found")
            # Publish plan marker to visualize in rviz
            self.publish_path()
            # remove initial waypoint in the path (current pose is already reached)= WANTING TO MOVE NEXT POSITION
            del self.path[0]     

        return self.path       
 
    # This method is called every 0.1s. It computes the velocity comands in order to reach the 
    # next waypoint in the path. It also sends zero velocity commands if there is no active path.

    def controller(self, event):
        v = 0
        w = 0
        if len(self.path) > 0:

            # TODO: If current waypoint is reached with some tolerance (0.1) move to the next waypoint.
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < 0.1:  

                del self.path[0]

                # If it was the last waypoint in the path show a message indicating it 
                if len(self.path) == 0:
                    self.goal = None
                    print("Final position reached!")
                    
            else: # TODO: Compute velocities using controller function in utils_lib (online_planning)
                v,w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        else:
            pass
        # # Publish velocity commands
        self.__send_commnd__(v, w)


    def recover(self):
        """
        Recovery behavior with backward motion
        """
        rospy.logwarn("Warning: Robot is currently stuck in an obstacle and attempting recovery.")
        cmd = Twist()
        start_time = rospy.Time.now()
        while (rospy.Time.now() -start_time ) < self.recovery_time:
            v = self.recovery_vel
            w = 0
            cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
            self.cmd_pub.publish(cmd)
        rospy.loginfo("Recovery completed.")
        cmd.linear.x = 0
        cmd.linear.y = 0
        self.cmd_pub.publish(cmd)


    def is_robot_in_obstacle(self):
        return not self.svc.is_valid(self.current_pose[0:2])

    # PUBLISHER HELPERS = IMPORTANT TO UNDERSTAND 
    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)

    # Publish a path as a series of line markers
    def publish_path(self):
        if len(self.path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in self.path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)
            
# MAIN FUNCTION
if __name__ == '__main__':
    rospy.init_node('turtlebot_online_path_planning_node')   
    node = OnlinePlanner('/projected_map', '/odom', '/cmd_vel', np.array([-10.0, 10.0, -10.0, 10.0]), 0.2)
    
    # Run forever
    rospy.spin()
