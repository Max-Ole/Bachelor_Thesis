# -*- coding: utf-8 -*-
"""
Created on Wed Dec 15 14:52:08 2021

@author: Dr. Ngoc Thinh Nguyen

Simulate a simple move-to-goal controller for a unicycle model
subject to pose-varying control limits
"""
import GridMapHelper

import sys
import numpy as np
import matplotlib.pyplot as plt
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from grid_map_msgs.msg import GridMap # type: grid_map_msgs/GridMap




class MapLimitedController():

	def __init__(self, T_step_size, mapManager):
		# listeners	& publishers

		rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalpose_callback)
		
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
		
		self.mapManager = mapManager

		self.dt = T_step_size
		self.goal_x = None
		self.goal_y = None
		self.threshold = 0.05
		#self.v_max = 0.5 # m/s
		#self.w_max = 0.2 # rad/s
		
		# store data
		"""x = np.zeros(Nsim + 1)
		y = np.zeros(Nsim + 1)
		yaw = np.zeros(Nsim +1)
		v = np.zeros(Nsim)
		w = np.zeros(Nsim)
		v_max_real = np.zeros(Nsim)
		a_max_real = np.zeros(Nsim)
		delta_angle = np.zeros(Nsim)
		x[0] = 1.0
		y[0] = 1.0
		yaw[0] = 3*np.pi/2"""
		
	def goalpose_callback(self, goal_msg):		
		""" Gets Position from Rviz. Rviz needs to have odom as fixed frame """
		self.goal_x = goal_msg.pose.position.x
		self.goal_y = goal_msg.pose.position.y
		
		
	def control_step(self, msg):
		"""Performs a control step. """		
		# need first to extract the control limits from the map
		v_map_limit = self.control_limit_extraction(x[i], y[i])
		# then calculate the control input
		(v_i, w_i, delta_angle_i) = self.move_to_goal_controller(x[i], y[i], yaw[i], v_map_limit, a_map_limit)
		(xnext, ynext, yawnext) = self.system_model(x[i], y[i], yaw[i], v_i, w_i)
		"""x[i+1] = xnext
		y[i+1] = ynext
		yaw[i+1] = yawnext
		v[i] = v_i
		w[i] = w_i
		delta_angle[i] = delta_angle_i
		v_max_real[i] = v_map_limit
		a_max_real[i] = a_map_limit"""
	
	def control_limit_extraction(self, x,y):
		"""
		Parameters (x,y) location
		----------
		Return the control limit, 
		taken from the map in real scenario
		"""
		a_limit = self.mapManager.getCellValue('the limit layer', *self.mapManager.pos2Cell(x, y))
		
		# Try with service, in order to use pre written c++ methods
		"""
		rospy.wait_for_service('readMapCell')
		try:
			readMapCell = rospy.ServiceProxy('readMapCell', AddTwoInts)
			resp1 = readMapCell(x, y)
			return resp1.sum
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)"""
		
		return a_limit


	def move_to_goal_controller(self, x, y, yaw, vmax, amax):
		K_w = 0.1
		K_v = 0.05
		# distance to goal
		d = np.sqrt((x-self.goal_x)**2 + (y-self.goal_y)**2) 
		# direction to the goal, in [-pi, pi]:
		yaw_ref = np.arctan2(self.goal_y-y, self.goal_x-x)
		# calculate the different angle and normalize it into [-pi, pi]:
		different_angle = np.arctan2(np.sin(yaw-yaw_ref), np.cos(yaw-yaw_ref))
		
		# a = (v2-v1) / dt  <=>  v2 = v1 + a*dt
		vmax_from_amax = abs( old_v + amax*self.dt )
		if d > self.threshold:
			# angular velocity
			w = - K_w * different_angle
			w = max(-0.5, min(0.5, w))
			# linear velocity
			v = K_v * d
			v = max(0., min(vmax, vmax_from_amax, v))
		else:
			v = 0.
			w = 0.
		return (v, w, different_angle)


	def system_model(self, x, y, yaw, v, w):
		xnext = x + self.dt * v * np.cos(yaw)
		ynext = y + self.dt * v * np.sin(yaw)
		yawnext = yaw + self.dt * w
		return (xnext, ynext, yawnext)

	def move(self, velocity, yaw):
		command =Twist()
		command.linear.x = velocity
		command.angular.z = yaw
		self.pub.publish(command)



	"""
		Plotting
	"""
	def plot_direction(self, x, y, tt, length = 0.1, ax=None, **kwargs):
		ax = ax or plt.gca()
		ax.arrow(x, y, length * np.cos(tt), length *np.sin(tt), **kwargs)

	def plot_trajectory(self):
		plt.figure()
		plt.scatter(x[0], y[1], color='red', s=20)
		plt.scatter(self.goal_x, self.goal_y, color='blue', s=20)
		plt.plot(x, y, 'r', label='simulation result')
		# ploting direction
		list_points = list(range(len(x)))
		list_points = list_points[0::10]
		for i in list_points:
			self.plot_direction(x[i], y[i], yaw[i], length = 0.5, linewidth = 0.5, color = 'b', head_width = 0.05)
		plt.legend(loc='best')
		plt.axis('equal')

		plt.figure()
		plt.plot(v, 'r', label='velocity')
		plt.plot(w, 'b', label='angular velocity')
		plt.plot(v_max_real, 'r--', label='velocity limit')
		plt.plot(a_max_real, 'b--', label='angular velocity limit')
		plt.legend(loc='best')

		plt.figure()
		plt.plot(delta_angle, 'r', label='heading angle to the goal')
		plt.legend(loc='best')

		plt.show()



def main(args):
	T_step_size = 0.2
	rospy.init_node("map_limit_controller", anonymous=True)
	
	mapManager = GridMapHelper.GridMapHelper()
	controller = MapLimitedController(T_step_size, mapManager)
	#plotter = GridMapHelper.SurfacePlotLive(mapManager)
	#try:
	rospy.sleep(0.5)
	#rospy.Timer(rospy.Duration(T_step_size), controller.control_step) # Set regular
	# live plot grid map
	#rospy.Timer(rospy.Duration(0.5), plotter.updateMap) # passing arg list
	rospy.spin()
	#except rospy.ROSInterruptException:
	#	controller.plot_trajectory()


if __name__=='__main__':
	main(sys.argv)
