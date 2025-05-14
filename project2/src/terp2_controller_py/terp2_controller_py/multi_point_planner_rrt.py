#!/usr/bin/env python3
# -*- coding: utf-8 -*-

""" Modified Version of A* Search Algorithm Multi-Point Planner
    to implement RRT* Search Algorithm
"""

# %%

#############################
# Dependencies
#############################
import pygame
import pygame.gfxdraw
import time
import math
import heapq
import numpy as np
import csv
import os
from rrt_star_planner import RRTStar, is_free 
from pathlib import Path

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



def round_and_get_v_index(node):
   #  Round x, y coordinates to nearest half to ensure our indexing is aligned 
   #  Round Theta to nearest 5 degrees
   
   x           = round(node[0] * 2, 1) / 2
   y           = round(node[1] * 2, 1) / 2
   theta_deg   = node[2]

   x_v_idx     = int(x * 2)
   y_v_idx     = int(y * 2)

   theta_deg_rounded = round(theta_deg / 5) * 5 # round to nearest 5 degrees
   theta_v_idx       = int(theta_deg_rounded % 360) // 5

   return (x, y, theta_deg_rounded), x_v_idx, y_v_idx, theta_v_idx

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

def ValidMove(node):
    # Check if move is valid
    if((node[0] < 0) or (node[0] >= 199)):
        return False
    elif((node[1] < 0) or (node[1] >= 200)):
        return False
    else:
        return True


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


def InObjectSpace(x, y, c_space):
    """
    Define the object space for all letters/numbers in the maze
    Also used for determining if a set of (x,y) coordinates is present in 
    the action space
    Returns: True if in Object Space, False if not
    """
    
    try: 
        if(c_space[(x,y)] == 0):
            return True
        else:
            return False
    except KeyError:
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
    return math.sqrt((goal_state[0] - node[0])**2 + (goal_state[1] - node[1])**2)


def DrawBoard(rows, cols, pxarray, pallet, C2C, clear, r, screen, c_space):
    """
    Draw the initial game board, colors depict:
    White: In object space
    Green: Buffer Zone
    Black: Action Space

    Turn clearance and robot radius used to calculate the total buffer zone that will be placed arround the walls and objects
    Robot will still be depicted on the screen as a "point-robot" as the center-point of the robot is the most import part for calculations

    Inputs:
        rows:    x-axis size
        cols:    y-axis size
        pxarray: pixel array for screen that allows for drawing by point
        pallet:  color dictionary with rgb codes
        clear:   clearance needed for turns in mm
        r:       robot raidus in mm

    Outputs:
        buffer_set: set of points that fall within buffer zone, 
                    used later for eliminating potential paths that navigate through the buffer zone 
                    to a point outside of the buffer zone and object space
    """
    buffer_set = set()
    buff_mod = clear + r
    
    # Ensure pixel array is wiped clean each time
    for x in range(0,rows):
        for y in range(0,cols):
            pxarray[x,y] = 0
    
    for x in range(1,rows-1):
        for y in range(0,cols):
            in_obj = InObjectSpace(x,y,c_space)
            if (in_obj):
                pygame.draw.circle(screen,pygame.Color(pallet["green"]),(x,y),buff_mod,0)
  
            elif(0<y<=buff_mod or (cols-buff_mod)<=y<cols):
                    pxarray[x,y] = pygame.Color(pallet["green"])

    for x in range(0,rows):
        for y in range(0,cols):
            if pxarray[x,y] == 0:
                pxarray[x,y] = pygame.Color(pallet["white"])

    for x in range(0,rows):
        for y in range(0,cols):
            if InObjectSpace(x,y, c_space):
                pxarray[x,y] = pygame.Color(pallet["black"])
    
    for x in range(0,rows):
        for y in range(0,cols):
            if screen.get_at((x,y)) != pygame.Color(pallet["white"]):
                buffer_set.add((x,y))

    return buffer_set



def FillCostMatrix(C2C, pxarray, pallet, thresh, rows, cols, screen):
    for x in range(0, int(rows/thresh)):
        for y in range(0, int(cols/thresh)):
            if((pxarray[int(math.floor(x*thresh)), int(math.floor(y*thresh))] == screen.map_rgb(pallet["black"])) or \
               (pxarray[int(math.floor(x*thresh)), int(math.floor(y*thresh))] == screen.map_rgb(pallet["green"]))):
                C2C[x,y] = -1
            else:
                C2C[x,y] = np.inf
                                 
def A_Star(start_node, goal_node, OL, parent, V, C2C, costsum, RPM1, RPM2, 
           t_curve,  pxarray, pallet, screen, goal_threshold, buffer_set,
           wheel_radius, wheel_base
           ):
    """
    Run A* Search algorithm over our board
    """

    solution_path = []
    
    start_state, x_v_idx, y_v_idx, theta_v_idx = round_and_get_v_index(start_node[1])
    start_cost_state = (x_v_idx, y_v_idx, theta_v_idx)
    C2C[x_v_idx, y_v_idx] = 0.0
    costsum[start_cost_state] = 0.0 + euclidean_distance(start_node[1], goal_node)

    heapq.heappush(OL, start_node)
    
    while OL:
        node = heapq.heappop(OL)

        # Take popped node and center it, along with finding the index values for the V matrix
        fixed_node, x_v_idx, y_v_idx, theta_v_idx = round_and_get_v_index(node[1])
        
        # Add popped node to the closed list
        V[x_v_idx, y_v_idx, theta_v_idx] = 1
        arc_end    = node[1]
        arc_speeds = node[2]
        arc_xy = reverse_move(arc_end, arc_speeds, t_curve, wheel_radius, wheel_base)

        pygame.draw.lines(screen,pygame.Color(pallet["blue"]),False,arc_xy,1)
        pygame.display.update()
        
        # Check if popped node is within the goal tolerance region
        # If so, end exploration and backtrace to generate the soluion path
        # If not, apply action set to node and explore the children nodes
        if(euclidean_distance(fixed_node, goal_node) <= goal_threshold ):
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
                test = move_set(fixed_node, action[0], action[1], buffer_set, t_curve, wheel_radius, wheel_base)
                if test is not None:

                    child_node, child_cost = test
                    
                    if not ValidMove(child_node): continue
                    
                    child_node_fixed, child_x_v_idx, child_y_v_idx, child_theta_v_idx = round_and_get_v_index(child_node)
                    child_cost_node = (child_x_v_idx, child_y_v_idx, child_theta_v_idx)
                    
                    # Check if node is in obstacle space or buffer zone
                    try:
                        if((pxarray[int(child_node_fixed[0]), int(child_node_fixed[1])] == screen.map_rgb(pallet["black"])) or \
                           (pxarray[int(child_node_fixed[0]), int(child_node_fixed[1])] == screen.map_rgb(pallet["green"]))): continue
                    except IndexError:
                        continue  # Attempted move was outside bounds of the map
                    
                    # Check if node is in visited list
                    # If not, check if in open list using cost matrix C2C
                    if(V[child_cost_node] == 0):
                        
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
# Initialize pygame

c_space = {}
csv_path = "mygrid.csv"
# if not os.path.isabs(csv_path):
#     csv_path = os.path.join(os.path.dirname(__file__), csv_path)

with open(csv_path, newline="") as f:
    next(f)
    
    for x, y, state in csv.reader(f):
        c_x = int((float(x)*10.0)+100)-1
        c_y = int((float(y)*10.0)+100)-1
        
        if (state == "unknown" or state == "occupied"):
            c_state = 0
        else: c_state = 1
        c_space[(c_x, c_y)] = c_state
        
   #rows, cols = c_x, c_y
    
#scale = 1

# Screen dimensions
rows, cols = (200, 200)

# Define Lists

TL = []  # Target List
SL = []  # Solution list
index_ctr  = 0
solution   = []
thresh     = 0.5
# wheel_radius = 50.0  # units: cm
# robot_radius = 30   # units: cm
# wheel_base   = 57.0 # units: cm

wheel_radius = 0.05  # units: cm
robot_radius = 0.300   # units: cm
wheel_base   = .570 # units: cm

t_curve        = 1 # seconds to run curve
goal_threshold = 5.01

theta_bins = 72

# Define colors
pallet = {"white":(255,  255, 255), 
          "black":(0,      0,   0), 
          "green":(4,    217,  13), 
          "blue": (61,   119, 245), 
          "red":  (242,   35,  24)
          }

# Define book rack goal points
dewey = {000: (65.0, 55.5, 180.0),
         100: (40.0, 35.5, 180.0),
         200: (165.0, 35.5, 0.0),
         300: (165.0, 85.5, 0.0),
         400: (165.0, 110.5, 0.0),
         500: (165.0, 135.5, 0.0),
         600: (70.0, 135.5, 180.0),
         700: (70.0, 95.5, 180.0)
         }

pygame.init()

start_node = [0.0, (99.0, 99.0, 0), (0,0)]
#goal_node  = (530.0, 149.0)
RPM1 = 5
RPM2 = 7

# Collect User Input for:
bookstack, n = GetUserInput()
TL = GetTargetPairs(dewey, bookstack, n, start_node)


# Define screen size
clearance   = 5
window_size = (rows+clearance, cols)
screen      = pygame.display.set_mode(window_size)
pxarray     = pygame.PixelArray(screen)


# Grab start time before running the solver
start_time = time.perf_counter()

# Start running solver
running = True
while running:
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    

    
    for targ in range(0,len(TL)):
    
        # Draw board with objects and buffer zone
        # rows:      size x-axis (named at one point, and forgot to change)
        # cols:      size y-axis
        # pallet:    color library with RGB values for drawing on pixel array
        # C2C:       obsolete, starting costs set in FillCostMatrix()
        # clearance: turn clearance for robot in mm 
        # rradius:   robot radius in mm
        
        V         = np.zeros((int(rows/thresh), int(cols/thresh), theta_bins))
        C2C       = np.zeros((int(rows/thresh), int(cols/thresh)))
        buffer_set = DrawBoard(rows, cols, pxarray, pallet, C2C, clearance, robot_radius, screen, c_space)

        # Update the screen
        pygame.display.update()
    
        FillCostMatrix(C2C, pxarray, pallet, thresh, rows, cols, screen)
        
        start, goal = TL[targ]
        OL = []
        CL = {}
        parent  = {}
        costsum = {}

        
        pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(start[1][0]), start[1][1]), radius=5.0, width=0) 
        pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(goal[0]), goal[1]), radius=5.0, width=1) 
        

        planner = RRTStar(
            start_xy=start[1][:2],          # (x, y)
            goal_xy=goal,                   # (x, y)
            is_free = lambda p: screen.get_at((int(p[0]), int(p[1]))) not in (
                      pygame.Color(pallet["black"]),
                      pygame.Color(pallet["green"])
                  ),
            sample_area=(0, rows, 0, cols),
            step_len=2.0,                   # tweak these three
            search_radius=10.0,
            goal_sample_rate=0.01,
            max_iter=4000
        )
        ok, rrt_path = planner.plan(display=screen)

        if ok:
            # wrap each pose in the same [(pos, speeds)] tuple shape A* used
            SL.append([((x, y, 0.0), (0, 0)) for x, y in rrt_path])
        else:
            print(f"RRT* failed for target {targ} after {planner.max_iter} samples")
            running = False
            break
    
        
        # Update the screen
        pygame.display.update()

        # after the for-loop (still inside `while running`)
    running = False          # all targets processed → exit solver loop
    end_time = time.perf_counter() 


# Calculate run time for solver
runtime = end_time - start_time

print("Time required to solve maze: ", runtime, " seconds")


center_y = 99
center_x = 99
final=[]
for i in range(0,len(SL)):
    temp = SL[i]
    for j in range(0,len(temp)):
        final.append(temp[j])
        
# directory that sits next to this script
path_folder = Path(__file__).resolve().parent / "path"
path_folder.mkdir(parents=True, exist_ok=True)   # ← create it if missing

with (path_folder / "rrt_path.csv").open("w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([RPM1, RPM2])
    for item in final:
        x = (item[0][0] - 99) / 10
        y = (item[0][1] - center_y ) / 10
        writer.writerow([x, y])

with (path_folder / "rrt_goals.csv").open("w", newline="") as file:
    writer = csv.writer(file)
    for item in TL:
        x = (item[1][0] - 99) / 10
        y = (item[1][1] - center_y ) / 10
        writer.writerow([x, y])

# # Draw start and goal points; start point will be filled, goal point will be hollow
pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(TL[0][0][1][0]), TL[0][0][1][1]), radius=5.0, width=0) # Start node    
pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(TL[n][1][0]), TL[n][1][1]), radius=5.0, width=1) # Goal node

# ## Draw solution path
final_path_xyt_list = []
final_path_drawing = []
for item in final:
    xyt = (int(round(item[0][0])),int(round(item[0][1])),int(round(item[0][2])))
    final_curve = reverse_move(xyt,item[1], wheel_radius, wheel_base, t_curve)
    
    pygame.draw.lines(screen,pygame.Color(pallet["red"]),False,final_curve,2)
    pygame.display.update()

# Freeze screen on completed maze screen until user quits the game
# (press close X on pygame screen)
running = True
while running:
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            # quit pygame
            pygame.quit()