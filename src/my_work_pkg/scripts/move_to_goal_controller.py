# -*- coding: utf-8 -*-
"""
Created on Wed Dec 15 14:52:08 2021

@author: Dr. Ngoc Thinh Nguyen

Simulate a simple move-to-goal controller for a unicycle model
subject to pose-varying control limits
"""

import numpy as np
import matplotlib.pyplot as plt

def control_limit_extraction(x,y):
    """
    Parameters (x,y) location
    ----------
    Return the control limit, 
    taken from the map in real scenario
    """
    v_limit = v_max + 0.8 * v_max * np.sin(2*x + y)
    w_limit = w_max + 0.8 * w_max * np.cos(x + 0.5*y)
    return (v_limit, w_limit)


def move_to_goal_controller(dt, x, y, yaw, xe, ye, threshold, vmax, wmax):
    K_w = 0.1
    K_v = 0.05
    # distance to goal
    d = np.sqrt((x-xe)**2 + (y-ye)**2) 
    # direction to the goal, in [-pi, pi]:
    yaw_ref = np.arctan2(ye-y, xe-x)
    # calculate the different angle and normalize it into [-pi, pi]:
    different_angle = np.arctan2(np.sin(yaw-yaw_ref), np.cos(yaw-yaw_ref))
    if d > threshold:
        # angular velocity
        w = - K_w * different_angle
        w = max(-wmax, min(wmax, w))
        # linear velocity
        v = K_v * d
        v = max(0., min(vmax, v))
    else:
        v = 0.
        w = 0.
    return (v, w, different_angle)


def system_model(dt, x, y, yaw, v, w):
    xnext = x + dt * v * np.cos(yaw)
    ynext = y + dt * v * np.sin(yaw)
    yawnext = yaw + dt * w
    return (xnext, ynext, yawnext)


Nsim = 1000
state0 = [1., 1, 3*np.pi/2]
xe = 3.
ye = 4.
threshold = 0.05
dt = 0.1
v_max = 0.5 # m/s
w_max = 0.2 # rad/s
# store data
x = np.zeros(Nsim + 1)
y = np.zeros(Nsim + 1)
yaw = np.zeros(Nsim +1)
v = np.zeros(Nsim)
w = np.zeros(Nsim)
v_max_real = np.zeros(Nsim)
w_max_real = np.zeros(Nsim)
delta_angle = np.zeros(Nsim)
x[0] = state0[0]
y[0] = state0[1]
yaw[0] = state0[2]
for i in range(Nsim):
    # need first to extract the control limits from the map
    (v_max_i, w_max_i) = control_limit_extraction(x[i], y[i])
    # then calculate the control input
    (v_i, w_i, delta_angle_i) = move_to_goal_controller(dt, x[i], y[i], yaw[i], xe, ye, threshold, v_max_i, w_max_i)
    (xnext, ynext, yawnext) = system_model(dt, x[i], y[i], yaw[i], v_i, w_i)
    x[i+1] = xnext
    y[i+1] = ynext
    yaw[i+1] = yawnext
    v[i] = v_i
    w[i] = w_i
    delta_angle[i] = delta_angle_i
    v_max_real[i] = v_max_i
    w_max_real[i] = w_max_i

"""
            Plotting
"""
def plot_direction(x, y, tt, length = 0.1, ax=None, **kwargs):
    ax = ax or plt.gca()
    ax.arrow(x, y, length * np.cos(tt), length *np.sin(tt), **kwargs)

plt.figure()
plt.scatter(x[0], y[1], color='red', s=20)
plt.scatter(xe, ye, color='blue', s=20)
plt.plot(x, y, 'r', label='simulation result')
# ploting direction
list_points = list(range(len(x)))
list_points = list_points[0::10]
for i in list_points:
    plot_direction(x[i], y[i], yaw[i], length = 0.5, linewidth = 0.5, color = 'b', head_width = 0.05)
plt.legend(loc='best')
plt.axis('equal')

plt.figure()
plt.plot(v, 'r', label='velocity')
plt.plot(w, 'b', label='angular velocity')
plt.plot(v_max_real, 'r--', label='velocity limit')
plt.plot(w_max_real, 'b--', label='angular velocity limit')
plt.legend(loc='best')

plt.figure()
plt.plot(delta_angle, 'r', label='heading angle to the goal')
plt.legend(loc='best')

plt.show()