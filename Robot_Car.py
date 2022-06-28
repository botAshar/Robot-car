#!/usr/bin/env python
# coding: utf-8

# In[1]:



# User Instructions
#
#The following code provides the filtering, planning, localization, smoothing functions
# for a robots moving in a 2 dimmentional grid as provided
#
 
from math import *
import random
from Class_plan import plan
from Robot_Class import robot
from Particle_Filter_Class import particles
from Parameter_Tuning import twiddle

# following are the noise paameters

steering_noise    = 0.1
distance_noise    = 0.03
measurement_noise = 0.3



# run:  runs control program for the robot
#


def run(grid, goal, spath, params, printflag = False, speed = 0.1, timeout = 1000):

    myrobot = robot()
    myrobot.set(0., 0., 0.)
    myrobot.set_noise(steering_noise, distance_noise, measurement_noise)
    filter = particles(myrobot.x, myrobot.y, myrobot.orientation,
                       steering_noise, distance_noise, measurement_noise)

    cte  = 0.0
    err  = 0.0
    N    = 0

    index = 0 # index into the path
    
    while not myrobot.check_goal(goal) and N < timeout:

        diff_cte = - cte

        # the present robot estimate
        estimate = filter.get_position()

        x = estimate[0]
        y = estimate[1]
        x1 = spath[index][0]
        x2 = spath[index+1][0]
        y1 = spath[index][1]
        y2 = spath[index+1][1]
        Rx = x - x1
        Ry = y - y1
        del_x = x2 - x1
        del_y = y2 - y1
        cte = (Ry * del_x - Rx * del_y) / ((del_x**2) + (del_y**2))
        
        #function for judging change of path of the robot
        u=(Rx * del_x + Ry * del_y)/((del_x**2) + (del_y**2))
        
        #if the paths changes then increase index while keeping it in range.
        if u>1 and (index+2)<len(spath):
            index+=1

        diff_cte += cte

        steer = - params[0] * cte - params[1] * diff_cte 

        myrobot = myrobot.move(grid, steer, speed)
        filter.move(grid, steer, speed)

        Z = myrobot.sense()
        filter.sense(Z)

        if not myrobot.check_collision(grid):
            print ('##### Collision ####')

        err += (cte ** 2)
        N += 1

        if printflag:
            print (myrobot, cte, index, u)

    return [myrobot.check_goal(goal), myrobot.num_collisions, myrobot.num_steps]


# ------------------------------------------------
# 
# this is our main routine
#

def main(grid, init, goal, steering_noise, distance_noise, measurement_noise, 
     weight_data, weight_smooth, p_gain, d_gain):

    path = plan(grid, init, goal)
    path.astar()
    path.smooth(weight_data, weight_smooth)
    return run(grid, goal, path.spath, [p_gain, d_gain])

    


# ------------------------------------------------
# 
# input data and parameters
#


# grid format:
#   0 = navigable space
#   1 = occupied space

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 0]]


init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]


steering_noise    = 0.1
distance_noise    = 0.03
measurement_noise = 0.3

weight_data       = 0.1
weight_smooth     = 0.2
p_gain            = 2.0
d_gain            = 6.0

weight_data, weight_smooth, p_gain, d_gain = twiddle([weight_data, weight_smooth, p_gain, d_gain])

    
print (main(grid, init, goal, steering_noise, distance_noise, measurement_noise, 
           weight_data, weight_smooth, p_gain, d_gain))







# In[ ]:




