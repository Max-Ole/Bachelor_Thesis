#!/usr/bin/env python2

import GridMapHelper

import rospy
import time
from geometry_msgs.msg import Twist
from math import copysign, sqrt, isnan
from copy import deepcopy


class ControlCapClass():

	def __init__(self, mapManager, control_input, control_output, safetyMargin=0.3):
		# listeners	& publishers
		self.vel_pub = rospy.Publisher(control_output, Twist, queue_size=2)
		rospy.Subscriber(control_input, Twist, self.cap_message)

		# given variables
		self.mapManager = mapManager
		self.control_input = control_input
		self.control_output = control_output
		self.safetyMargin = safetyMargin
		
		# init state variables
		self.last_t = time.time()
		self.last_v = 0
		
		# log
		self.log = ["time: seconds_since_epoche, state (zero_velocity/pass_through/capping), command.linear.x, command.linear.y, command.linear.z, command.angular.x, command.angular.y, command.angular.z, msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z\n"]
		
		
	def cap_message(self, msg):
		""" receives a control command and caps it to the value from the limit layer in the grid map (some safety margin in %).
		We only regard the "manhatten"-velocity and acceleration, not the 2-norm, from the grid map as tipping is considered around the axes aka. sides of the robot """
		command = deepcopy(msg)
		
		amax = (1-self.safetyMargin) * self.mapManager.getCellValue('a', *self.mapManager.pos2Cell(self.mapManager.mapPosition[1], self.mapManager.mapPosition[0]))
		vmax = (1-self.safetyMargin) * 100 # self.mapManager.getCellValue('v', *self.mapManager.pos2Cell(self.mapManager.mapPosition[1], self.mapManager.mapPosition[0]))
		
		# a = (v2-v1) / T_s  <=>  v2 = v1 + a*dt
		dt = time.time() - self.last_t
		vmax_from_amax = abs( self.last_v + amax*dt )
		# need to satisfy both limits
		v_limit_merged = float( min(vmax_from_amax, vmax) )
		
		if sum( [abs(msg.linear.x), abs(msg.linear.y), abs(msg.linear.z)] ) > 0.0001:
			# w = v/r  <=>  r = v/w  <=>  v = w*r
			# leave r as is, we do not want to change the robot's path. Also reduce velocity of all axes by the same factor
			reduction_ratio = v_limit_merged / max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.linear.z))
			if reduction_ratio >= 1 or isnan(v_limit_merged):
				# limits are not reached
				# pass Twist command unaltered
				self.vel_pub.publish(msg)
				
				# logging
				self.logMotion( time.time(), \
						"pass_through", \
						command.linear.x, \
						command.linear.y, \
						command.linear.z, \
						command.angular.x, \
						command.angular.y, \
						command.angular.z, \
						msg.linear.x, \
						msg.linear.y, \
						msg.linear.z, \
						msg.angular.x, \
						msg.angular.y, \
						msg.angular.z, \
						)
				return
		else:
			# velocity is zero, avoid division and
			# pass Twist command unaltered
			self.vel_pub.publish(msg)
			
			# logging
			self.logMotion( time.time(), \
						"zero_velocity", \
						command.linear.x, \
						command.linear.y, \
						command.linear.z, \
						command.angular.x, \
						command.angular.y, \
						command.angular.z, \
						msg.linear.x, \
						msg.linear.y, \
						msg.linear.z, \
						msg.angular.x, \
						msg.angular.y, \
						msg.angular.z, \
						)
			return
		
		assert reduction_ratio < 1, "ERROR: The reduction ratio ({}) when capping the control output should be smaller than 1".format(reduction_ratio)
		# copysign(x,y) returns x with the sign of y.
		#command.linear.x = copysign(min(v_limit_merged, abs(msg.linear.x)), msg.linear.x)
		#command.linear.y = copysign(min(v_limit_merged, abs(msg.linear.y)), msg.linear.y)
		#command.linear.z = copysign(min(v_limit_merged, abs(msg.linear.z)), msg.linear.z)
		
		# aply reduction ratio
		# change angular velocity to keep radius the same. Ratio of reduction must be the same
		command.linear.x  = reduction_ratio * msg.linear.x
		command.linear.y  = reduction_ratio * msg.linear.x
		command.linear.z  = reduction_ratio * msg.linear.x
		command.angular.x = reduction_ratio * msg.angular.x
		command.angular.y = reduction_ratio * msg.angular.y
		command.angular.z = reduction_ratio * msg.angular.z
		
		# update states 
		v_uncapped  = max(msg.linear.x, msg.linear.y, msg.linear.z)
		v_current   = max(command.linear.x, command.linear.y, command.linear.z)
		a_current   = abs(v_current - self.last_v) / dt
		self.last_v = v_current
		self.last_t = time.time()
		
		# print info # could change to 2norm for output: v_uncapped_2norm = sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
		print("capping acceleration: current accel: {}, cap: {}".format(a_current, amax))
		print("capping velocity:     current vel  : {}, cap: {}".format(v_current, vmax))
		
		# publish capped control
		self.vel_pub.publish(command)
		
		# logging
		self.logMotion( time.time(), \
						"capping", \
						command.linear.x, \
						command.linear.y, \
						command.linear.z, \
						command.angular.x, \
						command.angular.y, \
						command.angular.z, \
						msg.linear.x, \
						msg.linear.y, \
						msg.linear.z, \
						msg.angular.x, \
						msg.angular.y, \
						msg.angular.z, \
						)
		
		
	def logMotion(self, *args):
		""" Logs the data in CSV format """
		self.log.append("\n" + ",".join([str(i) for i in args]))
		
		
	def writeLog(self, filename):
		""" Writes the logged date to disk """
		outfile = open(filename, "w")
		outfile.writelines(self.log)
		print("Wrote log to "+filename+" .  "+str(len(self.log))+" lines.")
		outfile.close()


if __name__ == '__main__':
	try:
		global vel_pub, mapManager
		rospy.init_node('control_output_cap')
		
		control_input = rospy.get_param("~control_input")
		control_output = rospy.get_param("~control_output")

		mapManager = GridMapHelper.GridMapHelper()
		capObj = ControlCapClass(mapManager, control_input, control_output, safetyMargin=0.3)
		
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
	finally:
		capObj.writeLog( "/home/administrator/catkin_ws_Max-OleVW/src/my_work_pkg/scripts/log/logMotion_"+str(time.time())+".csv" )

