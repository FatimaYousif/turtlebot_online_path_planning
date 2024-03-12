import numpy as np
import random
import math

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

# everything about robot is in WORLD COORDINATES 
# MAP = was in SEM 1 - or - you convert it to one when you want to check if there´s an obstacle next.

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.3, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    # Given a pose, returs true if the pose is not in collision and false othewise.  
    def is_valid(self, pose, checking_path=False):
             

        # TODO: 1. convert world robot position to map coordinates using method __position_to_map__
        # TODO: 2. check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.

        # 1.
             grid_pose = self.__position_to_map__(pose)

             if self.__in_map__ == False:
                return False
             
             if grid_pose != []:

                grid_pose = (int(round(grid_pose[0],0)),int(round(grid_pose[1],0)))
                map_value = self.map[grid_pose]
                
                # 2. Return True if free, False if occupied and self.is_unknown_valid if unknown.
                if map_value == 0:   # If free space, check vicinity as well
                    return self.__check_vicinity__(grid_pose, self.distance,checking_path)
                    # return True
                elif map_value == -1 and self.is_unknown_valid == True:
                    return self.__check_vicinity__(grid_pose, self.distance,checking_path)
                    # return True
                else: # if obstacle, or if its unknown and is_unknown_valid = False, return False
                    return False
             else:
                return False


    # Given a path, returs true if the path is not in collision and false othewise.
          
    def check_path(self, path, step_size=0.04):

        # 1. TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # 2. TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True.

        waypoints = []  # upsampled points

        # 1. 
        for i in range(len(path)-1):
            p1, p2 = path[i], path[i+1]
            
            dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            
            num_steps = dist / step_size
            num_steps= int(num_steps)
            
            for j in range(num_steps):
                interpolation = float(j) / num_steps  #the interpolation value for each step to find the pt we are checking right now
                x = p1[0] * (1-interpolation) + p2[0] * interpolation
                y = p1[1] * (1-interpolation) + p2[1] * interpolation
                waypoints.append((x,y))

        # 2. 
        for w in waypoints:
            if self.is_valid(w,checking_path=True) == False:
                return False
        return True
    
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates.
        mx = (p[0]-self.origin[0])/self.resolution 
        my = (p[1]-self.origin[1])/self.resolution
        if self.__in_map__([mx,my]):
            return [mx,my]
        return [] 

    # ++++++++++++++++++++++++++++++++++++++++++++++++++
    def __in_map__(self, loc):
        '''
        loc: list of index [x,y]
        returns True if location is in map and false otherwise
        '''
        [mx,my] = loc
        if mx >= self.map.shape[0]-1 or my >= self.map.shape[1]-1 or mx < 0 or my < 0:
            return False 
        return True

    # ++++++++++++++++++++++++++++++++++++++++++++++++++
    def __check_vicinity__(self, cell, distance, checking_path=False):
        if checking_path:
            distance = distance/2
        discrete_distance = int(round(distance/self.resolution,0))
        for r in range(cell[0]-discrete_distance, cell[0]+discrete_distance):
            for c in range(cell[1]-discrete_distance, cell[1]+discrete_distance):
                if self.__in_map__([r,c]):
                    map_value = self.map[r,c]
                    if map_value == 100 or (map_value == -1 and self.is_unknown_valid == False): # if obstacle, or if its unknown and is_unknown_valid = False, return False
                        return False
        return True


# Define RRT class (you can take code from Autonopmous Systems course!)
class RRT:
    def  __init__(self, state_validity_checker, max_iterations=10000, delta_q=4, p_goal=0.2, dominion=[-10, 10, -10, 10]):
        # define constructor ...
        self.K = max_iterations
        self.delta_q = delta_q
        self.prob = p_goal
        self.min_x = dominion[0]
        self.max_x = dominion[1]
        self.min_y = dominion[2]
        self.max_y = dominion[3]
        self.parent = {}
        self.svc = state_validity_checker
    
    def compute_path(self, q_start, q_goal):
        # Implement RRT algorithm.
        # Use the state_validity_checker object to see if a position is valid or not.
        G = [q_start]
        parent = {}
        for k in range(self.K):
            qrand = self.create_random_point(self.prob, self.min_x, self.max_x, self.min_y, self.max_y, q_goal)
            qnear = self.find_nearest_node(qrand, G)

            while self.svc.is_valid(qnear) is False:
                 qnear = self.find_nearest_node(qrand, G)

            qnew = self.determine_new_node(qnear, qrand, self.delta_q)
            
            if self.svc.check_path([qnear,qnew], step_size=0.04):
                G.append(qnew)
                (self.parent)[tuple(qnew)] = tuple(qnear)
                if qnew[0] == q_goal[0] and qnew[1] == q_goal[1]:
                    path = self.construct_path(G, qnew)
                    path = self.smooth_path(path)
                    return path
        return []


    def smooth_path(self, path):
        # Optionally, you can implement a finction to smooth the RRT path.
        start_counter = 0
        goal_counter = len(path) - 1

        start = path[start_counter]
        goal = path[goal_counter]
        smooth_path = [goal]    # smoothing GOAL TO START
        while goal[0] != start[0] or goal[1] != start[1]:
            if self.svc.check_path([start,goal], step_size=0.04):
                smooth_path.insert(0, start)
                goal = path[start_counter]
                start_counter = 0
                start = path[start_counter]
            else:
                start_counter = start_counter + 1
                start = path[start_counter]
        return smooth_path
    
    # If you need any auxiliar method include it in this class

    # QRAND 
    def create_random_point(self, probability, xmin, xmax, ymin, ymax, goal_point):
        if random.random() > probability:
            random_point = random.randint(xmin, xmax), random.randint(ymin, ymax)
        else: 
            random_point = goal_point
        return random_point
    
    # Function to find the nearest node in the graph to a given point
    # QNEAR
    def find_nearest_node(self, random_point, graph):
        distances = [math.sqrt((random_point[0]-vertex[0])**2 + (random_point[1]-vertex[1])**2) for vertex in graph]
        min_index = distances.index(min(distances))
        return graph[min_index]
    
    # Function to determine a new node's position considering the step size
    # QNEW
    def determine_new_node(self, closest_node, random_point, step_size):
        distance = math.sqrt((random_point[0]-closest_node[0])**2 + (random_point[1]-closest_node[1])**2)
        if distance < step_size:
            return random_point
        return self.move_towards_point(step_size, closest_node, random_point)
    
    # Function to construct the path from start to goal
    def construct_path(self, graph, current_node):
        path = [list(current_node)]  # Start with the current_node as a list
        while tuple(current_node) in self.parent:
            current_node = self.parent[tuple(current_node)]
            path.insert(0, list(current_node))  # Convert current_node to a list and insert at the beginning
        return path

    
    # Function to move towards a point by a fixed distance
    def move_towards_point(self, step_size, start_point, end_point):
        if start_point[0] == end_point[0] and start_point[1] == end_point[1]:
            return start_point
        direction = (end_point[0] - start_point[0], end_point[1] - start_point[1])
        direction_length = math.sqrt(direction[0]**2 + direction[1]**2)
        normalized = (direction[0]/direction_length, direction[1]/direction_length)
        return (math.floor(start_point[0] + step_size * normalized[0]), math.floor(start_point[1] + step_size * normalized[1]))



# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, bounds, max_time=1.0):

    # TODO: Plan a path from start_p to goal_p inside bounds using the RRT and the 
    # StateValidityChecker Objects previously defined.
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    # example: [[x1, y1], [x2, y2], ...]
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!
    
    rrt = RRT(state_validity_checker, dominion=bounds)
    path = rrt.compute_path(start_p, goal_p)

    if path:
        return path
    else:
        return []


# DONE = ROS CRASHCOURSE
# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal. = DD robot
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # TODO: Use a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # To avoid strange curves, first correct the orientation and then the distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)

    d = math.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)
    psi_d = math.atan2(goal[1] - current[1], goal[0] - current[0])
    w = Kw * wrap_angle(psi_d - current[2])
    v = Kv* d
    # receiving a goal with curve -> stop the robot (zero linear velocity) and move  
    if wrap_angle(psi_d - current[2]) > 0.03:
        v = 0
    return v, w
