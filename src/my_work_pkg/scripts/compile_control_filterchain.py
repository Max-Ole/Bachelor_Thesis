#!/usr/bin/env python2

import rospy
import rosparam
import rospkg
#import sys
from math import sqrt, atan, asin, degrees




def calc_max_dynamic_accel(robot_config, axis):
	""" calculates the maximal acceleration that can be asserted in a direction given by axis by regarding the maximal centrifugal acceleration
v1 := wheel speed outer wheel
v2 := v1*xi wheel speed inner wheel, as factor xi in [-1,1] of other wheel
a = v1^2 / (2*wheels_dist) * (1-xi^2) 
Is maximal if v1=v_max and xi=0
==> a_max = v_max^2 / (2*wheels_dist) * 1 """
	assert axis in {x,y,z}, "axis must be one of 'x', 'y' or 'z'."
	# get highest v in direction perpendicular to given axes
	v_max = max({robot_config['velocity_max'][xyz] for xyz in {x,y,z} if xyz!=axis})
	
	# distance of wheels in direction of axis
	wheels_dist = robot_config['wheels_dist'][axis]
	#assert wheels_dist != 0, "No centrifugal acceleration can be calculated in {} direction. Wheel track is zero and robot should always tip over.".format(axis)
	if wheels_dist == 0:
		print "No centrifugal acceleration can be calculated in {} direction. Wheel track is zero and robot should always tip over.".format(axis)
		return float('inf')
	else:
		print "centrifugal acceleration in {}-direction: {}".format(axis, v_max**2 / (2*wheels_dist))
		return v_max**2 / (2*wheels_dist)


def get_robot_config():
	"""Loads a yaml file containing information about the robot."""
	filename = rospack.get_path(package)+"/robots/jackal_properties.yaml"
	robot_config = rosparam.load_file(filename, default_namespace=None, verbose=False)[0][0]
	return robot_config
	

def write_limits_filter_chain(robot_config):
	"""Saves the compiled limits to a yaml file used by the elevation_mapping and grid_map node. The limits will be used as part of the filter chain that post-processes the elevation map."""
	# reads the template and replaces placeholders ('<EDGE_INCLINE_THRESHOLD>','<SLOPE_LIMIT>','<MAX_SAFE_EDGE_HEIGHT>') with compiled limits
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing_limitMap_template.yaml", 'r') as f:
		content = f.read()
	
	"""
	criticalVal = {'a':{'a':None, 'l':None, 'h':None}, 'l':{'a':None, 'l':None, 'h':None}, 'h':{'a':None, 'l':None, 'h':None}}
	for axis in [x,y]:
		# Center of gravity and acceleration. static(?) or dynamic(?)
		a = max(robot_config['acceleration_max'][axis], calc_max_dynamic_accel(robot_config, axis))
		# distance from cog to closest wheel contact point
		l = robot_config['dist_cog_wheel'][axis]	# e.g. wheel[x] - cog[x]
		# height of cog to ground
		h =  robot_config['dist_cog_wheel'][z]	    # cog[z] - wheel[z]
		# gravitational constant
		g = 9.81
		
		b = sqrt(h**2 + l1**2)
		gamma = atan(l1/h)
		
		# TODO ????
		# keep most critical value for the most restrictive
		# keep the values that make the respective calculated value the safest
		# e.g. most restrictive a means most dangerous l,h
		if a < criticalVal['a']['a']: # smaller a is safer
			criticalVal['a']['a'] = a
			criticalVal['a']['l'] = l
			criticalVal['a']['h'] = h
		if l > criticalVal['l']['l']: # bigger l is safer
			criticalVal['l']['a'] = a
			criticalVal['l']['l'] = l
			criticalVal['l']['h'] = h
		if h < criticalVal['h']['h']: # smaller h is safer
			criticalVal['h']['a'] = a
			criticalVal['h']['l'] = l
			criticalVal['h']['h'] = h
	"""
	axis = x ## try on single axis first
	# Center of gravity and acceleration. static(?) or dynamic(?)
	a = max(robot_config['acceleration_max'][axis], calc_max_dynamic_accel(robot_config, axis))
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
	
	
	# save as filter chain
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing_limitMap.yaml", 'w') as f:
		f.write(content)


def main_program():
	""" Main function """
	
	robot_config = get_robot_config()
	write_limits_filter_chain(robot_config)


x, y, z = 'x', 'y', 'z' # z is assumed to point up or downwards
package = 'my_work_pkg'


if __name__ == '__main__':
	print("\nThis utility loads information about the robot from a yaml file, compiles it to limits of what obstacles can be traversed and saves these limits as another yaml file.\n")
	try:
		rospack = rospkg.RosPack()
		main_program()
	except rospy.ROSInterruptException:
		pass
