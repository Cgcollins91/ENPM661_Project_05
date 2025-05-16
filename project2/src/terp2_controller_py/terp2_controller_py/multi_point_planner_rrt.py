#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""  RRT* Search Algorithm
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
from pathlib import Path
from __future__ import annotations
from random import random, uniform
from math   import hypot



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


def move_set(node, u_l, u_r, buffer_set, wheel_radius, wheel_base, t_curve=2.0):
    """Simulate an arc for *t_curve* seconds; return ((x,y,θ), cost) or None."""
    t    = 0.0
    cost = 0.0
    dt   = t_curve / 10.0
    x_new, y_new, theta_new = node
    theta_new = math.radians(theta_new)
    u_l *= 2 * math.pi / 60.0        # RPM → rad s⁻¹
    u_r *= 2 * math.pi / 60.0

    while t < t_curve:
        t += dt
        v     = (wheel_radius * 0.5) * (u_r + u_l)
        x_new += v * math.cos(theta_new) * dt
        y_new += v * math.sin(theta_new) * dt
        theta_new += (wheel_radius / wheel_base) * (u_r - u_l) * dt
        cost += abs(v * dt)

        # obstacle hit? --------------------------------------------------
        if (int(math.ceil(x_new)), int(math.ceil(y_new))) in buffer_set or \
           (int(x_new), int(y_new)) in buffer_set:
            return None

    theta_new = math.degrees(theta_new) % 360.0
    return (x_new, y_new, theta_new), cost

# ──────────────────────────────────────────────────────────────────────────────
# Create a node class to hold the state of the robot
# (x, y, θ), control, and the cost to get to that state
# ──────────────────────────────────────────────────────────────────────────────

class Node:
    __slots__ = ("x", "y", "theta", "parent", "cost", "control")
    def __init__(self, x: float, y: float, theta: float = 0.0):
        self.x, self.y, self.theta = x, y, theta
        self.parent: Node | None = None
        self.cost   = 0.0          # Cost-To-Come
        self.control: tuple[int,int] | None = None   # (u_l, u_r)

    def dist(self, other: "Node") -> float:
        return hypot(self.x - other.x, self.y - other.y)

# ──────────────────────────────────────────────────────────────────────────────
# RRT* planner core
# ──────────────────────────────────────────────────────────────────────────────

class RRTStar:
    def __init__(self, *,
                 start_xy: tuple[float,float],
                 goal_xy:  tuple[float,float],
                 is_free,
                 sample_area: tuple[float,float,float,float],
                 step_len: float = 7.0,
                 search_radius: float = 20.0,
                 goal_sample_rate: float = 0.05,
                 max_iter: int = 3000,
                 RPM1: int = 5,
                 RPM2: int = 7,
                 wheel_radius: float = 0.05,
                 wheel_base: float = 0.57,
                 t_curve: float = 1.0,
                 buffer_set: set[tuple[int,int]] | None = None):

        if buffer_set is None:
            raise ValueError("buffer_set must be supplied for collision checking")

        self.start = Node(*start_xy)
        self.goal  = Node(*goal_xy)
        self.nodes: list[Node] = [self.start]

        self.is_free = is_free
        self.xmin, self.xmax, self.ymin, self.ymax = sample_area
        self.step_len, self.search_radius = step_len, search_radius
        self.goal_rate, self.max_iter = goal_sample_rate, max_iter
        self.RPM1, self.RPM2 = RPM1, RPM2
        self.wheel_radius, self.wheel_base = wheel_radius, wheel_base
        self.t_curve = t_curve
        self.buffer_set = buffer_set

    # ────────────────────────────────────────────────────────────────────
    def plan(self, display: pygame.Surface | None = None):
        if not self.is_free((self.start.x, self.start.y)) or not self.is_free((self.goal.x, self.goal.y)):
            return False, []

        surf = display
        for _ in range(self.max_iter):
            rnd = self._sample()
            nearest = self._nearest(rnd)
            new = self._steer(nearest, rnd)
            if new is None or not self._collision_free(nearest, new):
                continue

            # choose cheapest parent within radius ----------------------
            near_inds = self._near(new)
            parent    = nearest 
            min_cost  = nearest.cost + nearest.dist(new)
            for i in near_inds:
                nd = self.nodes[i]
                if self._collision_free(nd, new):
                    c = nd.cost + nd.dist(new)
                    if c < min_cost:
                        parent, min_cost = nd, c
            new.parent, new.cost = parent, min_cost
            self.nodes.append(new)

            # rewire -----------------------------------------------------
            for i in near_inds:
                nd = self.nodes[i]
                alt = new.cost + new.dist(nd)
                if alt < nd.cost and self._collision_free(new, nd):
                    nd.parent, nd.cost = new, alt

            if surf:
                pygame.draw.line(surf, (61,119,245), (new.x, new.y), (parent.x, parent.y))
                pygame.display.update()

            # goal reached? ---------------------------------------------
            if new.dist(self.goal) <= self.step_len:
                
                arc = move_set((new.x, new.y, new.theta),
                            *new.control,      # reuse the control that got to `new`
                            self.buffer_set,
                            self.wheel_radius,
                            self.wheel_base,
                            self.t_curve)
                if arc is not None:               # arc really is collision‑free
                    self.goal.parent = new
                    self.goal.cost   = new.cost + new.dist(self.goal)
                    return True, self._extract_path()
        return False, []

    # ─────────── helpers ───────────────────────────────────────────────
    def _sample(self):
        if random() < self.goal_rate:
            return (self.goal.x, self.goal.y)
        return (uniform(self.xmin, self.xmax), uniform(self.ymin, self.ymax))


    def _nearest(self, point):
        return min(self.nodes, key=lambda n: (n.x-point[0])**2 + (n.y-point[1])**2)


    def _steer(self, from_node: Node, to_xy):
        rpm_pairs = [(self.RPM1, self.RPM1), (self.RPM1, self.RPM2),
                     (self.RPM2, self.RPM1), (self.RPM2, self.RPM2)]
        best_pos, best_ctrl, best_d = None, None, float("inf")
        for ul, ur in rpm_pairs:
            res = move_set((from_node.x, from_node.y, from_node.theta), ul, ur,
                           self.buffer_set, self.wheel_radius, self.wheel_base, self.t_curve)
            if res is None:
                continue
            (x, y, th), _ = res
            d = hypot(x-to_xy[0], y-to_xy[1])
            if d < best_d:
                best_pos, best_ctrl, best_d = (x, y, th), (ul, ur), d
        if best_pos is None:
            return None
        node = Node(*best_pos)
        node.control = best_ctrl
        return node


    def _collision_free(self, n1: Node, n2: Node):
        if n2.control is None:  # straight‑line fallback
            steps = max(1, int(n1.dist(n2)))
            for i in range(steps+1):
                u = i/steps
                p = (n1.x + u*(n2.x-n1.x), n1.y + u*(n2.y-n1.y))
                if not self.is_free(p):
                    return False
            return True
        # simulate arc exactly as steer did
        return move_set((n1.x, n1.y, n1.theta), *n2.control, self.buffer_set,
                        self.wheel_radius, self.wheel_base, self.t_curve) is not None


    def _near(self, new: Node):
        r2 = self.search_radius ** 2
        return [i for i,n in enumerate(self.nodes) if (n.x-new.x)**2 + (n.y-new.y)**2 <= r2]


    def _extract_path(self):
        path = []
        node: Node | None = self.goal
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return list(reversed(path))

# ──────────────────────────────────────────────────────────────────────────────
# Check if a cell is free by cheking if  the pixel color is not black or green
# ──────────────────────────────────────────────────────────────────────────────

def is_free(p, pxarray, pallet, screen):
    x, y = int(p[0]), int(p[1])
    
    try:
        col = pxarray[x, y]
    except IndexError:
        return False

    return col not in (screen.map_rgb(pallet["black"]), screen.map_rgb(pallet["green"]))

    

    

#%%
# Initialize pygame

c_space = {}
csv_path = "mygrid.csv"


with open(csv_path, newline="") as f:
    next(f)
    
    for x, y, state in csv.reader(f):
        c_x = int((float(x)*10.0)+100)-1
        c_y = int((float(y)*10.0)+100)-1
        
        if (state == "unknown" or state == "occupied"):
            c_state = 0
        else: c_state = 1
        c_space[(c_x, c_y)] = c_state
        


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
    
     
        
        start, goal = TL[targ]
        OL = []
        CL = {}
        parent  = {}
        costsum = {}

        
        pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(start[1][0]), start[1][1]), radius=5.0, width=0) 
        pygame.draw.circle(screen, pygame.Color(pallet["red"]), (int(goal[0]), goal[1]), radius=5.0, width=1) 
        
        t_curve = 20 # seconds to run curve
        planner = RRTStar(
            start_xy=start[1][:2],          # (x, y)
            goal_xy=goal,                   # (x, y)
            is_free = lambda p: screen.get_at((int(p[0]), int(p[1]))) not in (
                      pygame.Color(pallet["black"]),
                      pygame.Color(pallet["green"])
                  ),
            sample_area=(0, rows, 0, cols),
            step_len=4.0,                   # tweak these three
            search_radius=14.0,
            goal_sample_rate=0.02,
            max_iter=6000,
            RPM1=RPM1,
            RPM2=RPM2,
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
            t_curve=t_curve,
            buffer_set=buffer_set,
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

with (path_folder / "path.csv").open("w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([RPM1, RPM2])
    for item in final:
        x = (item[0][0] - 99) / 10
        y = (item[0][1] - center_y ) / 10
        writer.writerow([x, y])

with (path_folder / "goals.csv").open("w", newline="") as file:
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