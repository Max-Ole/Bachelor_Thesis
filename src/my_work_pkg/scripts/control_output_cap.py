#!/usr/bin/env python2

import GridMapHelper

import rospy
import rospkg
import time
from geometry_msgs.msg import Twist
from math import sqrt, isnan
from copy import deepcopy
from math import sqrt, tan, atan, asin, cos, degrees

import compile_robot_limitations


class ControlCapClass():

	def __init__(self, mapManager, control_input, control_output, robot_config, safetyMargin=0.3):
		self.robot_config = robot_config
		
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
		
		if sum( [abs(msg.linear.x), abs(msg.linear.y), abs(msg.linear.z)] ) < 0.01:
			# velocity is zero, avoid division and
			# pass Twist command unaltered
			self.vel_pub.publish(msg)
			
			self.last_v = 0
			self.last_t = time.time()
			
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
		
		# velocity is non-zero
		amax = (1-self.safetyMargin) * self.mapManager.getCellValue('a', *self.mapManager.pos2Cell(self.mapManager.mapPosition[1], self.mapManager.mapPosition[0]))
		if amax <= 0: 
			# invalid limit, could be a sensor error
			amax = float('nan') # alternative: halt robot
		vmax = (1-self.safetyMargin) * self.maxVelFromCentripetalAccel(msg, amax) # 100
		
		# a = (v2-v1) / T_s  <=>  v2 = v1 + a*dt
		dt = time.time() - self.last_t
		vmax_from_amax = abs(self.last_v) + amax*dt
		vmin_from_amax = max(0, abs(self.last_v) - amax*dt)
		# need to satisfy both limits
		v_limit_merged = float( min(vmax_from_amax, vmax) )

		reduction_ratio = v_limit_merged / max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.linear.z)) # upper bound
		increase_ratio  = vmin_from_amax / max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.linear.z)) # lower bound
		
		# take no action if: limit is nan  or  not up to date  or  msg is within limits
		if isnan(v_limit_merged) or dt>=1 or (reduction_ratio >= 1 and increase_ratio <= 1) :
			# msg is within limits
			# pass Twist command unaltered
			self.vel_pub.publish(msg)
			
			self.last_v = max(msg.linear.x, msg.linear.y, msg.linear.z)
			self.last_t = time.time()
			
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
			if increase_ratio >= 1:
				# lower limit exceeded
				print( "lower limit exceeded" )
				ratio = increase_ratio
			elif reduction_ratio <= 1:
				# upper limit exceeded
				print( "upper limit exceeded" )
				ratio = reduction_ratio
			
			# aply reduction ratio
			# change angular velocity to keep radius r the same, we do not want to change the robot's path. Also reduce velocity of all axes by the same factor
			# w = v/r  <=>  v = w*r  <=>  r = v/w = (v*ratio)/(w*ratio)
			command.linear.x  = ratio * msg.linear.x
			command.linear.y  = ratio * msg.linear.x
			command.linear.z  = ratio * msg.linear.x
			command.angular.x = ratio * msg.angular.x
			command.angular.y = ratio * msg.angular.y
			command.angular.z = ratio * msg.angular.z
			
			# update states 
			v_uncapped  = max(msg.linear.x, msg.linear.y, msg.linear.z)
			v_current   = max(command.linear.x, command.linear.y, command.linear.z)
			a_uncapped  = (v_uncapped - self.last_v) / dt
			a_current   = (v_current - self.last_v) / dt
			
			# print info # could change to 2norm for output: v_uncapped_2norm = sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
			print("\tchanging acceleration from {:6.2f} to {:6.2f}, cap: {:6.2f}".format(a_uncapped, a_current, amax))
			print("\tchanging velocity (last:{:6.2f}) from {:6.2f} to {:6.2f}, between: {:6.2f} and {:6.2f}".format(v_uncapped, self.last_v, v_current, vmin_from_amax, v_limit_merged))
			print("\tratio: {}, dt: {}".format(ratio, dt))
			
			self.last_v = v_current
			self.last_t = time.time()
			
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
	
	def maxVelFromCentripetalAccel(self, msg, amax):
		""" Gets the max velocity from the maximal allowed acceleration and the current radius. From compile_robot_limitations.py:
a_ges = lambda r_c: v_max**2 * r_CoG(r_c) / r_1(r_c)**2 * cos(theta_1(r_c))**2 * cos(theta_CoG(r_c))
v_max = a_ges * r_1(r_c)**2 / ( r_CoG(r_c) * cos(theta_1(r_c))**2 * cos(theta_CoG(r_c)) )"""
		r = self.radiusFromTwist(msg)
		
		CoG_x = self.robot_config['center_of_gravity'][x]
		CoG_y = self.robot_config['center_of_gravity'][y]
		wheel1_x = self.robot_config['wheel3'][x]
		wheel1_y = self.robot_config['wheel3'][y]
		
		r_CoG     = lambda r_c: sqrt( (r_c - CoG_y)**2    + (CoG_x)**2    )
		r_1       = lambda r_c: sqrt( (r_c - wheel1_y)**2 + (wheel1_x)**2 )
		theta_CoG = lambda r_c: atan( abs(float(CoG_x))    / abs(r_c - CoG_y)    )
		theta_1   = lambda r_c: atan( abs(float(wheel1_x)) / abs(r_c - wheel1_y) )
		
		r_c = 0.264088029343 # worst r_c as determined by compile_robot_limitations.py. Could use current curve radius.
		
		v_max = amax * r_1(r_c)**2 / ( r_CoG(r_c) * cos(theta_1(r_c))**2 * cos(theta_CoG(r_c)) )
		
		return v_max
	
	def radiusFromTwist(self, msg):
		""" Gets the radius of the current trajectory
v=omega*r <=> r=v/omega <=> omega=v/r """
		v = ( msg.linear.x**2 + msg.linear.y**2 )**0.5  # assume msg.linear.z is nearly 0
		omega = msg.angular.z
		if abs(omega) > 0.01:
			r = v/omega
			return r
		else:
			# avoid division by 0
			return None # float('inf')
		
		
	def logMotion(self, *args):
		""" Logs the data in CSV format """
		self.log.append("\n" + ",".join([str(i) for i in args]))
		
		
	def writeLog(self, filename):
		""" Writes the logged date to disk """
		outfile = open(filename, "w")
		outfile.writelines(self.log)
		print("Wrote log to "+filename+" .  "+str(len(self.log))+" lines.")
		outfile.close()


x, y, z = 'x', 'y', 'z'
package = 'my_work_pkg'

if __name__ == '__main__':
	global vel_pub, mapManager
	rospy.init_node('control_output_cap')
	rospack = rospkg.RosPack()
	
	control_input = rospy.get_param("~control_input")
	control_output = rospy.get_param("~control_output")
	
	try:
		mapManager = GridMapHelper.GridMapHelper()
		robot_config = compile_robot_limitations.get_robot_config()
		capObj = ControlCapClass(mapManager, control_input, control_output, robot_config, safetyMargin=0.3)
		
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
	finally:
		capObj.writeLog( rospack.get_path(package)+"/scripts/log/logMotion_"+str(time.time())+".csv" ) 
		# on robot: /home/administrator/catkin_ws_Max-OleVW/src/my_work_pkg/scripts/log/logMotion_

