# %%

import os
import csv
import cv2
from matplotlib import pyplot as plt
import numpy as np

values = []
with open('mygrid.csv','r', newline='') as csvfile:
    csv_reader = csv.reader(csvfile)
    for row in csv_reader:
        x,y,state = row
        if state == "occupied":
            values.append((float(x),float(y)))

csvfile.close()


def xy_to_cv2_scaled(xy):
    x,y = xy
    x = int(round(10*x))
    y = int(round(10*y))
    y = int(100-y)
    x = int(100+x)
    return (x,y)

def xy_to_cv2_idx(xy):
    x,y = xy
    x = int(round(10*x))
    y = int(round(10*y))
    y = int(100-y)
    x = int(100+x)
    return y,x

map = np.zeros((201,201,1),dtype=np.uint8)
h,w,_ = map.shape

for item in values:

    map[xy_to_cv2_idx(item)] = 255

#cv2.HoughLinesP()
lines = cv2.HoughLinesP(map,rho=1,theta=np.deg2rad(1),threshold=30,minLineLength=60,maxLineGap=4)
print(len(lines))


obstacle_map = np.zeros((h,w,1),dtype=np.uint8)
buffer_map = obstacle_map.copy()
for line in lines:
    x1 = line[0][0]
    y1 = line[0][1]
    x2 = line[0][2]
    y2 = line[0][3]
    obstacle_map = cv2.rectangle(obstacle_map,(x1,y1-5),(x2,y2+5),255,-1)


obstacle_map = cv2.rectangle(obstacle_map,(0,0),(w-1,h-1),255,1)

robot_radius = 3
clearance = 5
buffer_size = robot_radius+clearance
obs_set = set()
obs_coord = np.where(obstacle_map==255)
for i in range(0,len(obs_coord[0])):
    xy = (obs_coord[1][i],obs_coord[0][i])
    obs_set.add(xy)

for item in obs_set:
    buffer_map = cv2.circle(buffer_map,item,buffer_size,255,-1)

buffer_set = set()
buffer_coord = np.where(buffer_map==255)
for i in range(0,len(buffer_coord[0])):
    xy = (buffer_coord[1][i],buffer_coord[0][i])
    if xy not in obs_set:
        buffer_set.add(xy)


display_map = np.zeros((h,w,3),dtype=np.uint8)
for item in obs_set:
    display_map[item[1],item[0]] = (255,0,0)

for item in buffer_set:
    display_map[item[1],item[0]] = (0,255,0)

for item in values:
    display_map[xy_to_cv2_idx(item)] = (0,0,255)

out_file = 'display_map.csv'
h, w, _ = display_map.shape

with open(out_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    # note: y first so it matches image coordinates (row = y, col = x)
    for y in range(h):
        for x in range(w):
            b, g, r = display_map[y, x]             # OpenCV stores BGR
            # only save non-black pixels (optional – comment out if you
            # want every pixel)
            if b == 255:
                writer.writerow([y, x, 100])
            elif g == 255:
                writer.writerow([y, x, 100])
            else:
                writer.writerow([y, x, 0])


cv2.imshow('map',display_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
# %%
