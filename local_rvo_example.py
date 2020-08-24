import sys
import cv2 as cv

from RVO import RVO_update, reach, compute_V_des, reach,Tools
from vis import visualize_traj_dynamic


#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.2
#circular obstacles, format [x,y,rad]
# no obstacles
#ws_model['circular_obstacles'] = []
# with obstacles
#ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
ws_model['circular_obstacles'] = [[4.25,2.25,0.2],
                                  [1.5,0.3,0.2],[1.5,0.5,0.2],[1.5,0.7,0.2],[1.5,0.8,0.2],[1.5,0.9,0.2],
                                  [7.0,4.2,0.2],[7.0,4.0,0.2],[7.0,3.8,0.2],[7.0,3.7,0.2],[7.0,3.6,0.2],
                                  [0.3,3.5,0.2],[0.5,3.5,0.2],[0.7,3.5,0.2],[0.8,3.5,0.2],[0.9,3.5,0.2],
                                  [8.2,1.0,0.2],[8.0,1.0,0.2],[7.8,1.0,0.2],[7.7,1.0,0.2],[7.6,1.0,0.2],
                                  [1.4,2.25,0.2],[1.6,2.25,0.2],[1.7,2.25,0.2],[1.9,2.25,0.2],[2.1,2.25,0.2],
                                  [1.4+5,2.25,0.2],[1.6+5,2.25,0.2],[1.7+5,2.25,0.2],[1.9+5,2.25,0.2],[2.1+5,2.25,0.2],
                                  [8.2-3.6,0.9,0.2],[8.0-3.6,0.9,0.2],[7.8-3.6,0.9,0.2],[7.7-3.6,0.9,0.2],[7.6-3.6,0.9,0.2],
                                  [0.3+3.6,3.6,0.2],[0.5+3.6,3.6,0.2],[0.7+3.6,3.6,0.2],[0.8+3.6,3.6,0.2],[0.9+3.6,3.6,0.2]]
#rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = []

#------------------------------
#initialization for robot 
# position of [x,y]
X = [[2.735148514851485, 0.504672897196262],[8.0,0.5],[8.0,4.0],[0.5,4.0]]
# velocity of [vx,vy]
V = [[0,0] for i in range(len(X))]
# maximal velocity norm
V_max = [1.0 for i in range(len(X))]
# goal of [x,y]

#goals = [[[1.5,1.5],[5.5,0.8],[7.0,3.0],[3.0,3.7]],[[8.0,4.0],[0.5,4.0],[0.5,0.5],[8.0,0.5]]]
path = [[65, 95], [79, 85.10344827586206], [76.79347826086956, 71.10344827586206], [90.79347826086956, 69.20866978631264], [104.79347826086956, 69.47115583195502], [118.79347826086956, 69.73364187759739], [132.79347826086956, 69.99612792323977], [133, 70], [147, 81.2], [161, 92.4], [163, 94]]
goals = [Tools.pix2world(path),
         [[5.5,0.8],[0.5,4.0]],
         [[7.0,3.0],[0.5,0.5]],
         [[3.0,3.7],[8.0,0.5]]]
#------------------------------
#simulation setup
# total simulation time (s)
total_time = 15
# simulation step
step = 0.05

#------------------------------
#simulation starts
t = 0
tool = Tools(4)

while t*step < total_time:
    # compute desired vel to goal
    goal = tool.get_goal(X,goals)
    print(tool.visited)
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model,local=2)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    if t%2 == 0:
        data = visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10),save=True)
        cv.imshow('image',data)
        cv.waitKey(1)
        #visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
    t += 1
    
