#!/usr/bin/env python2

import rospy
import rosparam
import rospkg
#import sys
from math import sqrt, atan, asin, degrees

from compile_robot_limitations import *



def max_dynamic_accel(robot_config, axis):
	""" calculates the maximal acceleration that can be asserted in a direction given by axis by regarding the maximal centrifugal acceleration when turning. Considers position of center of gravity (CoG) and an outer wheel. These need to be described in a frame such that the rotation centers x-value is 0. """
	
	assert axis in {x,y,z}, "axis must be one of 'x', 'y' or 'z'."
	# get variables
	v_max = max({robot_config['velocity_max'][xyz] for xyz in {x,y,z} if xyz!=axis}) # get highest v in direction perpendicular to given axes
	if v_max < 0.001:
		print("No centrifugal acceleration in {}-direction as robot can't move perpendicular to it".format(axis,))
		return 0.0
	
	CoG_x = robot_config['center_of_gravity'][x]
	CoG_y = robot_config['center_of_gravity'][y]
	wheel1_x = robot_config['wheel'][x]
	wheel1_y = robot_config['wheel'][y]
	
	# Define formulas
	r_CoG     = lambda r_c: sqrt( (r_c + CoG_y)**2    + (CoG_x)**2    )
	r_1       = lambda r_c: sqrt( (r_c + wheel1_y)**2 + (wheel1_x)**2 )
	theta_CoG = lambda r_c: atan( float(CoG_x)    / (r_c + CoG_y)    )
	theta_1   = lambda r_c: atan( float(wheel1_x) / (r_c + wheel1_y) )
	
	a_ges = lambda r_c: v_max**2 * r_CoG(r_c) / r_1(r_c)**2 * cos(theta_1(r_c))**2 * cos(theta_CoG(r_c))
	
	r_c_values   = np.linspace(0, 3, 3000)
	a_ges_values = np.zeros(r_c_values.shape)
	a_ges_max = -float('inf')
	r_c_worst = None
	for idx,i in enumerate(r_c_values):
		a_ges_values[idx] = a_ges(i)
		if a_ges_values[idx] > a_ges_max:
			a_ges_max = a_ges_values[idx]
			r_c_worst = i

	print "centrifugal acceleration in {}-direction is maximal at radius {} with value {}".format(axis, r_c_worst, a_ges_max)
	return a_ges_max


def get_robot_config():
	"""Loads a yaml file containing information about the robot."""
	filename = rospack.get_path(package)+"/robots/jackal_properties.yaml"
	robot_config = rosparam.load_file(filename, default_namespace=None, verbose=False)[0][0]
	return robot_config
	

def write_limits_filter_chain(robot_config, values):
	"""Saves the compiled limits to a yaml file used by the elevation_mapping and grid_map node. The limits will be used as part of the filter chain that post-processes the elevation map."""
	# reads the template and replaces placeholders ('<EDGE_INCLINE_THRESHOLD>','<SLOPE_LIMIT>','<MAX_SAFE_EDGE_HEIGHT>') with compiled limits
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing_limitMap_template.yaml", 'r') as f:
		content = f.read()
	
	"""
	criticalVal = {'a':{'a':None, 'l':None, 'h':None}, 'l':{'a':None, 'l':None, 'h':None}, 'h':{'a':None, 'l':None, 'h':None}}
	for axis in [x,y]:
		# Center of gravity and acceleration. static(?) or dynamic(?)
		a = max(robot_config['acceleration_max'][axis], max_dynamic_accel(robot_config, axis))
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
	a = max(robot_config['acceleration_max'][axis], max_dynamic_accel(robot_config, axis))
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
	content = content.replace('<EDGE_INCLINE_THRESHOLD>', str(values['slope'])) # TODO could be different than slope?
	content = content.replace('<MAX_SAFE_EDGE_HEIGHT>', str(values['edge']))
	
	# save as filter chain
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing_limitMap.yaml", 'w') as f:
		f.write(content)


def main_program():
	""" Main function """
	
	robot_config = get_robot_config()
	robot_limits = compile2limits(robot_config)
	write_limits_filter_chain(robot_config, robot_limits)


x, y, z = 'x', 'y', 'z' # z is assumed to point up or downwards
package = 'my_work_pkg'


if __name__ == '__main__':
	print("\nThis utility loads information about the robot from a yaml file, compiles it to limits of what obstacles can be traversed and saves these limits as another yaml file.\n")
	try:
		rospack = rospkg.RosPack()
		main_program()
	except rospy.ROSInterruptException:
		pass
