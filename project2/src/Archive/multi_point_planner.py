#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################
# Dependencies
#############################
import time
import math
import heapq
import numpy as np
import csv
import os, csv, math, heapq, time, numpy as np
import rclpy
from rclpy.node   import Node
from rclpy.qos    import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg       import Float64MultiArray
from nav_msgs.msg       import Path
from geometry_msgs.msg  import PoseStamped

CSV_FILE = "map.csv"       # param-ise if you like



def CheckOpenList(coords, open_list):
    '''
    Check if coords is in the Open List
    Pops all items from OL, checks each to see if coordinates have already
    been visited, appends popped item to temp_list, then puts all items 
    back into OL
    Returns: True is present, False if not
    '''
    temp_list = []
    present = False
    
    while open_list:
        item = heapq.heappop(open_list)
        if item[1] == coords:
            present = True
        heapq.heappush(temp_list, item)
        
    return present, temp_list


def CheckClosedList(coords, closed_list):
    """
    Check if x_prime is in the Closed List
    Closed list is a dictionary with each node's coords used as a key
    If x_prime coordinates are not in the closed list, a KeyError is thrown, 
    which is caught by the try-except block.
    Returns: True if present, False if not
    """
    
    try:
        if(closed_list[coords]):
            return True
    except KeyError:
        return False





def move_set(node, u_l, u_r, buffer_set, wheel_radius, wheel_base, t_curve=2):
    """
    Utilizes function that was provided in Cost.py of the Proj 3 Phase 2 files. 
    Get potential moves running action for t_curve seconds
    Variable names have been adjusted slightly, but general mechanics remain the same.
    """
    t    = 0
    cost = 0
    dt   = t_curve / 10
    
    x_new = node[0]
    y_new = node[1]

    theta_new = 3.14 * node[2] / 180 # Convert to Radians
    u_l       = u_l*2*math.pi/60     # Convert to Rad/s
    u_r       = u_r*2*math.pi/60    
    while t < t_curve:
        t = t+dt
        x_new     += (wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new     += (wheel_radius * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        theta_new += (wheel_radius/wheel_base) * (u_r - u_l) * dt
        cost       = cost + math.sqrt(math.pow(((wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((wheel_radius * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        
        # Check if move takes us into buffer around wall
        bcux       = int(math.ceil(x_new))
        bcuy       = int(math.ceil(y_new))
        if bcux <0 or bcuy < 0:
            return None
        bcdx = int(x_new)
        bcdy = int(y_new)
        if bcdx < 0 or bcdy < 0:
            return None
        if (bcux,bcuy) in buffer_set or (bcdx,bcdy) in buffer_set:
  
            return None
    
    theta_new = int(180 * theta_new / 3.14) # Convert back to degrees

    
    return (x_new, y_new, theta_new), cost



def reverse_move(node,movement, wheel_radius, wheel_base, t_curve):
    # Reverse Move for Curve Plotting
    cost  = 0
    dt    = -t_curve/10
    x_new = node[0]
    y_new = node[1]
    u_l = movement[0]
    u_r = movement[1]
    u_l = u_l*2*math.pi/60
    u_r = u_r*2*math.pi/60    
    theta_new = 3.14 * node[2] / 180
    xy_list = [(x_new,y_new)]
    t=0
    #print("reverse Theta Start in rad: ", theta_new)

    
    while t > -t_curve:
        t = t+dt
        theta_new += (wheel_radius/wheel_base) * (u_r - u_l) * dt
        x_new += (wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt
        y_new += (wheel_radius * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt
        
        cost = cost + math.sqrt(math.pow(((wheel_radius * 0.5)*(u_r + u_l) * math.cos(theta_new)*dt),2) + math.pow(((wheel_radius * 0.5)*(u_r + u_l) * math.sin(theta_new)*dt),2))
        xy_list.append((round(x_new),round(y_new)))
        
    theta_new = int(180 * theta_new / 3.14)

    return xy_list


def InObjectSpace(x, y, CELL_SIZE=.05):
    """
    Define the object space for all letters/numbers in the maze
    Also used for determining if a set of (x,y) coordinates is present in 
    the action space
    Returns: True if in Object Space, False if not
    """
    MAP_SIZE     = int(20 // CELL_SIZE)        
    right_most_x = MAP_SIZE - 1
    half_way_x   = int(MAP_SIZE // 2)
    half_way_y   = int(MAP_SIZE // 2)

    # Object 1
    if ((0<=x<=right_most_x) and (0<=y<=99)):
        return True
    
    # Object 2
    elif ((0<=x<=half_way_x) and (399<=y<=499)): 
        return True
    
    # Object 3
    elif ((1499<=x<=right_most_x) and (399<=y<=499)):
        return True
    
    # Object 4
    elif((0<=x<=half_way_x) and (half_way_y<=y<=1099)):
        return True

    # Object 5
    elif((1499<=x<=right_most_x) and (half_way_y<=y<=1099)):
        return True
    
    # Object 6
    elif((0<=x<=1999) and (1399<=y<=1499)):
        return True
    
    # Object 7 -- Desk Near Start Point
    elif((0<=x<=199) and (549<=y<=949)):
        return True

    # Define Object Space for walls
    elif( ( (0<=x<=1999) and y==0) or ((0<=x<=1999) and y==1499)):# or (x==0 and (0<=y<=699)) or (x==1399 and (0<=y<=699)) ): \
        return True

    # Default case, non-object space    
    else:
        return False



def GeneratePath(CurrentNode, parent, start_state):
    """
    Backtrace the solution path from the goal state to the initial state
    Add all nodes to the "solution" queue
    """
    solution = []
    backtracing = True
    
    solution.append(CurrentNode)
    node = parent[CurrentNode]
    
    while backtracing:
        if(node[0] == start_state):
            solution.append(node)
            backtracing = False
        else:
            solution.append(node)
            node = parent[node]
            
    solution.reverse()
    return solution


def euclidean_distance(node, goal_state):
    """
    Calculate Euclidean Distance between current node and goal state
    Euclidean Distance is the straight line distance between two points
    distance metric used in A* Search
    """
    return math.sqrt((goal_state[0] - node[0])**2 + (goal_state[1] - node[1])**2)*10



def FillCostMatrix(C2C, map, thresh, rows, cols):
    for x in range(0, int(rows/thresh)):
        for y in range(0, int(cols/thresh)):
            if((map[int(math.floor(x*thresh)), int(math.floor(y*thresh))] == 100) ) :
                C2C[x,y] = -1
            else:
                C2C[x,y] = np.inf
    return C2C
                                 





def GetUserInput():
    bookstack = []
    done = False
    stack_index = 0
    
    while not done:
        print("Welcome to the library, let's get those books back on the shelves!\n")
        n = int(input("How many books do you have? (1-6): "))
        
        if( n <= 0 or n > 6):
            print("Sorry, that's an invalid number of books. Please try again.\n")
            continue
        
        while stack_index < n:
            print("Enter an int between 0 - 799 for the index of book #%d: " % (stack_index + 1))
            dd_index = int(float(input()))  # Cast to float first before casting, in case user gets creative with their dewey decimal index values (otherwise string to int causes ValueError)
            
            if(dd_index < 0 or dd_index >= 800):
                print("That is an invalid index, please try again!")
                continue
            else:
                bookstack.append(dd_index)
                stack_index += 1
                
        done = True

    return bookstack, n
# End UserInput()

def GetTargetPairs(dewey, bookstack, n, start_node):
    targets = []
    t_pairs = []
    
    # Find bin coordinates for each book that needs to be reshelved
    # Place in priority queue (heapq) so that the books can be
    # returned in order of bin number
    for i in range(0,n):
        book_idx = int(np.floor(bookstack[i]/100) * 100)
        bin_num = (book_idx/100)
        heapq.heappush(targets, (bin_num, dewey[book_idx]))
    
    # Assemble target pairs to build path from start to each bin in order
    node_i = start_node
    while targets:
        goal_coords = targets[0][1][:2]
        t_pairs.append((node_i, goal_coords))
        heading = heapq.heappop(targets)
        node_i = [0.0, heading[1], (0,0)]
    
    # Close the return loop by creating a pair that travels from last bin visited
    # back to start point
    t_pairs.append((node_i, start_node[1][:2]))

    return t_pairs
    
    
# End GetTargets
    

#%%
class MultiPointPlanner(Node):
    def __init__(self) -> None:
        super().__init__("multi_point_planner")

        # ---- parameters --------------------------------------------------
        self.declare_parameter("csv_file", CSV_FILE)
        self.declare_parameter("start", [0.0, 0.0, 0.0])   # x,y,θ°
        self.declare_parameter("goal",  [2.0, 1.5, 0.0])
        self.declare_parameter("rpm1",  10.0)
        self.declare_parameter("rpm2",  15.0)

        # ---- publishers --------------------------------------------------
        self.path_pub = self.create_publisher(
            Path, "/planned_path",
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.wp_pub = self.create_publisher(
            Float64MultiArray, "/waypoints", 1)

        # ---- compute once on startup ------------------------------------
        self.compute_and_publish()

    
    def round_and_get_v_index(self, node, scale):
        #  Round x, y coordinates to nearest half to ensure our indexing is aligned 
        #  Round Theta to nearest 5 degrees
        
        x           = round(node[0] * scale, 1) / scale 
        y           = round(node[1] * scale, 1) / scale 
        theta_deg   = node[2]

        x_v_idx     =  int(math.floor(x * scale))
        y_v_idx     =  int(math.floor(y * scale))

        x_v_idx  = max(0, min(x_v_idx , self.WIDTH  - 1))
        y_v_idx  = max(0, min(y_v_idx , self.HEIGHT - 1))
            # ---------------------------------

        theta_deg_rounded = round(theta_deg / 5) * 5 # round to nearest 5 degrees
        theta_v_idx       = int(theta_deg_rounded % 360) // 5

        return (x, y, theta_deg_rounded), x_v_idx, y_v_idx, theta_v_idx

    def ValidMove(self, node):
        # Check if move is valid
        if((node[0] < 0) or (node[0] >= self.HEIGHT)):
            self.get_logger().info(f"***")
            return False
        elif((node[1] < 0) or (node[1] >= self.WIDTH)):
            self.get_logger().info(f"***")
            return False
        else:
            return True

    def A_Star(self, start_node, goal_node, OL, parent, V, C2C, costsum, RPM1, RPM2, 
            t_curve,  map, obstacles, scale, goal_threshold,
            wheel_radius, wheel_base
            ):
        """
        Run A* Search algorithm over our board
        """

        solution_path = []
        
        start_state, x_v_idx, y_v_idx, theta_v_idx = self.round_and_get_v_index(start_node[1], scale)
        goal_state, x_v_idx2, y_v_idx2, theta_v_idx2 = self.round_and_get_v_index(goal_node, scale)
        start_cost_state = (x_v_idx, y_v_idx, theta_v_idx)
        C2C[x_v_idx, y_v_idx] = 0.0
        costsum[start_cost_state] = 0.0 + euclidean_distance(start_node[1], goal_node)

        heapq.heappush(OL, start_node)
        
        while OL:
            node = heapq.heappop(OL)

            # Take popped node and center it, along with finding the index values for the V matrix
            fixed_node, x_v_idx, y_v_idx, theta_v_idx = self.round_and_get_v_index(node[1], scale)
            self.get_logger().info(f"Node: {fixed_node}")
            
            # Add popped node to the closed list
            V[x_v_idx, y_v_idx, theta_v_idx] = 1
            arc_end    = node[1]
            arc_speeds = node[2]
            arc_xy = reverse_move(arc_end, arc_speeds, t_curve, wheel_radius, wheel_base)

            
            # Check if popped node is within the goal tolerance region
            # If so, end exploration and backtrace to generate the soluion path
            # If not, apply action set to node and explore the children nodes
            if(euclidean_distance(fixed_node, goal_state) <= goal_threshold ):
                solution_path = GeneratePath((fixed_node, arc_speeds), parent, start_state)
                return True, solution_path
            else:
                actions = [[0.0,  RPM1],
                        [RPM1,  0.0],
                        [RPM1, RPM1],
                        [0.0,  RPM2],
                        [RPM2,  0.0],
                        [RPM2, RPM2],
                        [RPM1, RPM2],
                        [RPM2, RPM1]]
                
                # Walk through each child node created by action set and determine if it has been visited or not
                for action in actions:
                    #if move_set(fixed_node, action[0], action[1]) is not None:
                    test = move_set(fixed_node, action[0], action[1], obstacles, t_curve, wheel_radius, wheel_base)
                    if test is not None:

                        child_node, child_cost = test
                        
                        if not self.ValidMove(child_node): continue
                        
                        child_node_fixed, child_x_v_idx, child_y_v_idx, child_theta_v_idx = self.round_and_get_v_index(child_node, scale)
                        child_cost_node = (child_x_v_idx, child_y_v_idx, child_theta_v_idx)
                        
                        # Check if node is in obstacle space or buffer zone
                        try:
                            if((map[int(child_node_fixed[0]), int(child_node_fixed[1])] == 100 )): continue
                        except IndexError:
                            continue  # Attempted move was outside bounds of the map
                        
                        # Check if node is in visited list
                        # If not, check if in open list using cost matrix C2C
                        if(V[child_cost_node] == 0):
                            # self.get_logger().info(f"Exploring: {child_cost_node}")
                            
                            # If child is not in open list, create new child node
                            if(C2C[child_x_v_idx, child_y_v_idx]  == np.inf):
                                cost2go = round(euclidean_distance(child_node_fixed, goal_node),2)  # Calculate Cost to Go using heuristic equation Euclidean Distance
                                cost2come = C2C[x_v_idx, y_v_idx] + child_cost  # Calculate Cost to Come using parent Cost to Come and step size
                                parent[(child_node_fixed, (action[0],action[1]))] = (fixed_node, arc_speeds)     # Add child node to parent dictionary 
                                
                                C2C[child_x_v_idx, child_y_v_idx] = cost2come   # Update cost matrix with newly calculate Cost to Come
                                costsum[child_cost_node] = cost2come + cost2go  # Calculate the total cost sum and add to reference dictionary (this will be used when determiniing optimal path)
                                child = [costsum[child_cost_node], child_node_fixed,(action[0],action[1])]  # Create new child node --> [total cost, (x, y, theta)]... Total cost is used as priority determinant in heapq
                                heapq.heappush(OL, child)   # push child node to heapq
                                
                        # Child was in visited list, see if new path is most optimal
                        else:
                            cost2go = round(euclidean_distance(child_node_fixed, goal_node),2)
                            cost2come = C2C[x_v_idx, y_v_idx] + child_cost
                            
                            # Compare previously saved total cost estimate to newly calculated
                            # If new cost is lower than old cost, update in cost matrix and reassign parent 
                            if(costsum[child_cost_node] > (cost2come + cost2go)):  
                                parent[(child_node_fixed, (action[0],action[1]))] = (fixed_node, arc_speeds)
                                C2C[child_x_v_idx, child_y_v_idx] = cost2come
                                costsum[child_cost_node] = cost2come + cost2go
                                
        return False, solution_path


    # =====================================================================

    def compute_and_publish(self):

        # 1. read CSV ------------------------------------------------------
        obstacles = []
        WIDTH, HEIGHT = 200, 200
        csv_path  = self.get_parameter("csv_file").value
        map       = np.zeros((HEIGHT, WIDTH), np.uint8)
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(os.path.dirname(__file__), csv_path)

        with open(csv_path, newline="") as f:
            for x, y, state in csv.reader(f):
                if state == 100:
                    # obstacles.append(x,y)
                    map[y,x] = 100
        scale = 1
        map  = np.kron(map,
                   np.ones((scale, scale), dtype=map.dtype))
        (self.WIDTH, self.HEIGHT) = map.shape

        for ix, iy in zip(*np.where(map == 100)):
            obstacles.append((iy, ix))

        rpm1  = self.get_parameter("rpm1").value
        rpm2  = self.get_parameter("rpm2").value
        start = self.get_parameter("start").value
        goal  = self.get_parameter("goal").value
        thresh = 1
        theta_bins = 72
        TL = []  # Target List
        SL = []  # Solution list
        V         = np.zeros((int(self.HEIGHT/thresh), int(self.WIDTH/thresh), theta_bins))
        C2C       = np.zeros((int(self.HEIGHT/thresh), int(self.WIDTH/thresh)))       


        C2C = FillCostMatrix(C2C, map, thresh, self.WIDTH, self.HEIGHT)
        
        OL = []
        CL = {}
        parent  = {}
        costsum = {}
        start_node = [0.0, (0.0, 0.0, 0.0), (0,0)]

        # 2. build obstacle / buffer sets exactly like before --------------
        # buffer_set, obstacle_set = self.build_buffer_sets(obstacles)

        # 3. run your A* ---------------------------------------------------


        # Define book rack goal points
        dewey = {000: (-6.5, 5.5, 90.0),
                100: (-8.0,  1.5, 90.0),
                200: (7.5,   1.5, 90.0),
                300: (6.5,   5.5, 90.0),
                400: (6.5,   9.5, 270.0),
                500: (6.5,  -3.5, 270.0),
                600: (-7.0, -3.5, 270.0),
                700: (-7.0, -6.5, 270.0)
                }

        bookstack, n = GetUserInput()
        TL           = GetTargetPairs(dewey, bookstack, n, start_node)
       

        for start, goal in TL:
            start_xy_theta = start[1]
            
            start_node = [0.0,
              ( int(start_xy_theta[0] * scale)+ (self.HEIGHT // 2)*.1, int(start_xy_theta[1] * scale)+ (self.HEIGHT // 2)*.1, start_xy_theta[2] ),
              (0,0)]
            self.get_logger().info(f"Start: {start_node}")
            goal = (goal[0]+(self.HEIGHT // 2)*.1, goal[1]+(self.HEIGHT // 2)*.1, 0)

            self.get_logger().info(f"Goal: {goal}")
            goal_node, x_v_idx, y_v_idx, theta_v_idx = self.round_and_get_v_index(goal, scale)

            success, solution = self.A_Star(
                start_node,
                goal, OL, parent, V, C2C, costsum,
                5, 7,
                t_curve=3.0,
                # dummy placeholders (no Pygame)
                map=map, obstacles=obstacles, scale = scale,
                goal_threshold=10,
                wheel_radius=.5, wheel_base=0.57)
            
            SL.append(solution)

            if not success:
                self.get_logger().error("Path-planner: no path found.")
                return

        # 4. publish nav_msgs/Path ----------------------------------------
        path_msg = Path()
        path_msg.header.frame_id = "path_plan"
        for i, (pose, _) in enumerate(SL):
            p = PoseStamped()
            p.header.frame_id = "path_plan"
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = pose[0] / 100.0    # convert back to meters
            p.pose.position.y = pose[1] / 100.0
            path_msg.poses.append(p)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published nav_msgs/Path with {len(solution)} pts")

        # 5. publish way-points as Float64MultiArray ----------------------
        flat = []
        for pose, _ in SL:
            flat.extend([pose[0] / 100.0, pose[1] / 100.0])
        self.wp_pub.publish(Float64MultiArray(data=flat))
        self.get_logger().info("Published way-point array")

def main():
    rclpy.init()
    node = MultiPointPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
