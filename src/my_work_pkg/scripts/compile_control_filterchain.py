#!/usr/bin/env python2

import rospy
import rosparam
import rospkg
#import sys
from math import sqrt, atan, asin, degrees
import compile_robot_limitations


def compile_to_control_limits(robot_config):
	"""Compiles information about the robot to determine the limits of what obstacles can be traversed. """
	### inclination max
	angle_limits = {'tipping over': float('inf'), \
					'motor stalling': float('inf'), \
					'slippage': float('inf') \
					}
	# tipping over
	angle_limits['tipping over'] = max_incline_tipping_0accel(robot_config)
	
	# motor stalling
	angle_limits['motor stalling'] = compile_robot_limitations.max_incline_torque(robot_config) # previously robot_config['motor_max_incline']
	
	# slippage due to low friction
	angle_limits['slippage'] = robot_config['slip_tolerance_max_incline']
	
	# TODO add front or back of robot hitting ramp
	# return limit and output the reason for that limit aka. the most restrictive design choice for traversability.
	reason = min(angle_limits, key=lambda k: angle_limits[k])
	print "\nThe threat of {} is the most restrictive for the highest safely traversable inclination.\n".format(reason)
	
	
	### edge max
	edge_limits = {'edge experiment': float('inf'), 'footprint': 0}
	# must be smaller than ground clearance
	edge_limits['edge experiment'] = min(robot_config['edge_max'], robot_config['ground_clearance'])
	# The window in which edges are searched must surround the robot
	edge_limits['footprint'] = compile_robot_limitations.conv_window_size(robot_config, edge_limits['edge experiment'], angle_limits[reason])
	
	
	robot_limits = {'slope': angle_limits[reason], \
					'edge': edge_limits['edge experiment'], \
					'footprint_for_edge_determination': edge_limits['footprint']}
	return robot_limits


def max_incline_tipping_0accel(robot_config):
	""" Calculates the maximal inclination for all axes that can safely be traversed with the given geometry and ZERO acceleration.
For each axis the inclination is found so that the robot just starts to tip over. That is the point where the moments around the tipping point (closest wheel) due to gravity and acceleration are equal. 
Formula as in max_incline_tipping but simplified with a=0
alpha = gamma - asin( (a*h)/(g*b) )  ==> alpha = gamma = atan(l1/h)"""

	#accel_max = max( robot_config['acceleration_max'].values() )
	critical_alpha = float('inf')
	critical_axis = None
	
	for axis in [x,y]:
		# distance from cog to closest wheel contact point
		l1 = robot_config['dist_cog_wheel'][axis]	# e.g. wheel[x] - cog[x]
		# height of cog to ground
		h =  robot_config['dist_cog_wheel'][z]	    # cog[z] - wheel[z]
		
		alpha = atan(l1/h) # =gamma
		
		# keep lowest, most critical value
		print "Maximal safe inclination for axis {}: {}".format(axis, degrees(alpha))
		if alpha < critical_alpha:
			critical_alpha = alpha
			critical_axis = axis
		
	print "The {}-axis is the most restrictive in terms of not tipping over in the static case.\n".format(critical_axis)
	return critical_alpha
	

def write_limits_filter_chain(robot_config, values):
	"""Saves the compiled limits to a yaml file used by the elevation_mapping and grid_map node. The limits will be used as part of the filter chain that post-processes the elevation map."""
	# reads the template and replaces placeholders (e.g. '<EDGE_INCLINE_THRESHOLD>','<SLOPE_LIMIT>',...) with compiled limits
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing_limitMap_template.yaml", 'r') as f:
		content = f.read()
	
	axis = x ## try on single axis first
	# Center of gravity and acceleration. max of static or dynamic
	a = max(robot_config['acceleration_max'][axis], compile_robot_limitations.max_dynamic_accel(robot_config, axis))
	# distance from cog to closest wheel contact point
	l = robot_config['dist_cog_wheel'][axis]	# e.g. wheel[x] - cog[x]
	# height of cog to ground
	h =  robot_config['dist_cog_wheel'][z]	    # cog[z] - wheel[z]
	# gravitational constant
	g = 9.81
	
	b = sqrt(h**2 + l**2)
	gamma = atan(l/h)
	
	content = content.replace('<g>', str(g))
	content = content.replace('<a_4h>', str(a)).replace('<a_4b>', str(a))
	content = content.replace('<b_4h>', str(b)).replace('<b_4a>', str(b))
	content = content.replace('<h_4a>', str(h)).replace('<h_4b>', str(h))
	content = content.replace('<gamma_4a>', str(gamma)).replace('<gamma_4b>', str(gamma)).replace('<gamma_4h>', str(gamma))
	
	# standard changes as in normal compile_robot_limitations.py
	content = content.replace('<SLOPE_LIMIT>', str(values['slope']))
	content = content.replace('<EDGE_INCLINE_THRESHOLD>', str(values['slope'])) # Extension posibility: could be different than slope
	content = content.replace('<MAX_SAFE_EDGE_HEIGHT>', str(values['edge']))
	content = content.replace('<EDGE_FOOTPRINT>', str(values['footprint_for_edge_determination']))
	
	# save as filter chain
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing_limitMap.yaml", 'w') as f:
		f.write(content)


def main_program():
	""" Main function """
	
	robot_config = compile_robot_limitations.get_robot_config()
	
	# calc limits for fundamental obstacles. Obstacles regardless of behavior ==> static case of acceleration and speed are 0
	robot_limits = compile_to_control_limits(robot_config)
	
	write_limits_filter_chain(robot_config, robot_limits)


x, y, z = 'x', 'y', 'z' # z is assumed to point up or downwards
package = 'my_work_pkg'


if __name__ == '__main__':
	print("\nThis utility loads information about the robot from a yaml file, compiles it to limits of what obstacles can be traversed and saves these limits as another yaml file. \nHere we assume the BEST possible robot motion, because we assume a controller that can obey control limits we provide.\n\n")
	try:
		rospack = rospkg.RosPack()
		main_program()
	except rospy.ROSInterruptException:
		pass
