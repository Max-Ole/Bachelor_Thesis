#!/usr/bin/env python2

import numpy as np
import rospy
import rosparam
import rospkg
from math import sqrt, tan, atan, asin, cos, degrees



def compile2limits(robot_config):
	"""Compiles information about the robot to determine the limits of what obstacles can be traversed. """
	### inclination max
	angle_limits = {'tipping over': float('inf'), \
					'motor stalling': float('inf'), \
					'slippage': float('inf') \
					}
	# tipping over
	angle_limits['tipping over'] = max_incline_tipping(robot_config)
	
	# motor stalling
	angle_limits['motor stalling'] = max_incline_torque(robot_config) # previously robot_config['motor_max_incline']
	
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
	edge_limits['footprint'] = conv_window_size(robot_config, edge_limits['edge experiment'], angle_limits[reason])
	
	
	robot_limits = {'slope': angle_limits[reason], \
					'edge': edge_limits['edge experiment'], \
					'footprint_for_edge_determination': edge_limits['footprint']}
	return robot_limits


def conv_window_size(robot_config, e_lim, angle_lim):
	""" Determines the windows size used by a convolution in the filter chain. There exist the following constraints:
	- square
	- as small as possible for precision
	- at least the size of robot footprint
	- large enough that no edges are found along a slope at inclination of the slope limit
	"""
	footprint_square = max(robot_config['wheels_dist'][x], robot_config['wheels_dist'][y])
	c_lim = float(e_lim) / tan(angle_lim)
	print("Convolution Window size is {} meters. It is the maximum of the footprint square ({} m) and the minimal size ({} m) due to the slope limit.".format(max(footprint_square, c_lim), footprint_square, c_lim))
	
	return max(footprint_square, c_lim)
	

def max_incline_torque(robot_config):
	""" Calculates the maximal inclination that can safely be traversed with the given the robot's total motor torque, mass and wheel radius. 
	Derived from: 
	T = r*F where r and F are orthogonal and F=m*g*sin(alpha) is the force needed to hold the robot at the inclination alpha. """
	
	# radius of driven wheels
	r = robot_config['wheel_radius']
	# mass
	m = robot_config['mass']
	# gravitational constant
	g = 9.81
	
	# total motor torque. Read from file or calculate
	try:
		T = float(robot_config['torque']) # sum of torque of all motors
	except (TypeError, ValueError):
		# torque was not defined in the yaml file
		# calculate torque from wattage, wheel radius and speed
		W = robot_config['watt']
		v_max = max(robot_config['velocity_max'].values())
		T = float(r*W) / v_max
		print("Calculating torque from wattage as no torque was provided: torque={}Nm".format(T))
	
	if abs( float(T) / (r*m*g) ) >=1:
		print("The motor torque in this configuration (torque:{}Nm, mass:{}kg and wheel_radius:{}m) is enough to drive up a wall vertically. (Given that the tires interlock with the wall, of course)".format(round(T,3), round(m,3), round(r,3)))
		return asin(1.0)
		
	return asin( float(T) / (r*m*g) )


def max_incline_tipping(robot_config):
	""" Calculates the maximal inclination for all axes that can safely be traversed with the given geometry and worst possible acceleration and trajectory.
For each axis the inclination is found so that the robot just starts to tip over. That is the point where the moments around the tipping point (closest wheel) due to gravity and acceleration are equal. """
	#accel_max = max( robot_config['acceleration_max'].values() )
	critical_alpha = float('inf')
	critical_axis = None
	
	for axis in [x,y]:
		# Center of gravity and acceleration. static(?) or dynamic(?)
		a = max(robot_config['acceleration_max'][axis], max_dynamic_accel(robot_config, axis))
		# distance from cog to closest wheel contact point
		l1 = robot_config['dist_cog_wheel'][axis]	# e.g. wheel[x] - cog[x]
		# height of cog to ground
		h =  robot_config['dist_cog_wheel'][z]	    # cog[z] - wheel[z]
		# gravitational constant
		g = 9.81
		
		b = sqrt(h**2 + l1**2)
		gamma = atan(l1/h)

		try:
			alpha = gamma - asin( (a*h)/(g*b) )
		except ValueError: # accelerating at max acceleration would tip robot regardless of inclination
			print "ERROR: Robot seems inherently unstable. Please check the robot property values. E.g. the max acceleration could be too high.\n"
			raise ValueError
		if alpha<0.0: # max acceleration only doesn't tip robot if accelerating downhill
			print "ERROR: Robot seems inherently unstable. Please check the robot property values. E.g. the max acceleration could be too high.\n"
			raise ValueError
		
		# keep lowest, most critical value
		print "Maximal safe inclination for axis {}: {}".format(axis, degrees(alpha))
		if alpha < critical_alpha:
			critical_alpha = alpha
			critical_axis = axis
		
	print "The {}-axis is the most restrictive in terms of not tipping over due to center of gravity, stance, and acceleration.\n".format(critical_axis)
	return critical_alpha


def max_dynamic_accel(robot_config, axis):
	""" calculates the maximal acceleration that can be asserted in a direction given by axis by regarding the maximal centrifugal acceleration when turning. Considers position of center of gravity (CoG) and an outer wheel. These need to be described in a frame such that the rotation centers x-value is 0. 
We assume that turning in either direction is symetric and we consider turning left (positive radius). This is the worst case for the Jackal as its cog is a bit to the right"""
	
	assert axis in {x,y,z}, "axis must be one of 'x', 'y' or 'z'."
	# get variables from yaml
	v_max = max({robot_config['velocity_max'][xyz] for xyz in {x,y,z} if xyz!=axis}) # get highest v in direction perpendicular to given axes
	if v_max < 0.001:
		print("No centrifugal acceleration in {}-direction as robot can't move perpendicular to it".format(axis,))
		return 0.0
	
	CoG_x = robot_config['center_of_gravity'][x]
	CoG_y = robot_config['center_of_gravity'][y]
	wheel1_x = robot_config['wheel3'][x]
	wheel1_y = robot_config['wheel3'][y]
	minimal_turning_radius = robot_config['minimal_turning_radius']
	
	# Define formulas
	r_CoG     = lambda r_c: sqrt( (r_c - CoG_y)**2    + (CoG_x)**2    )
	r_1       = lambda r_c: sqrt( (r_c - wheel1_y)**2 + (wheel1_x)**2 )
	theta_CoG = lambda r_c: atan( abs(float(CoG_x))    / abs(r_c - CoG_y)    )
	theta_1   = lambda r_c: atan( abs(float(wheel1_x)) / abs(r_c - wheel1_y) )
	
	a_ges = lambda r_c: v_max**2 * r_CoG(r_c) / r_1(r_c)**2 * cos(theta_1(r_c))**2 * cos(theta_CoG(r_c))
	
	# find largest a_ges over range of r_c
	r_c_values   = np.linspace(0, 3, 3000)
	a_ges_values = np.zeros(r_c_values.shape)
	a_ges_max = -float('inf')
	r_c_worst = None
	for idx,i in enumerate(r_c_values):
		a_ges_values[idx] = a_ges(i)
		if a_ges_values[idx] > a_ges_max:
			a_ges_max = a_ges_values[idx]
			r_c_worst = i
	
	# regard minimal_turning_radius of robot. Might overwrite r_c and thus a_ges maximum
	if r_c_worst < minimal_turning_radius:
		r_c_worst = minimal_turning_radius
		a_ges_max = a_ges(r_c_worst)
		
	print "Centrifugal acceleration in {}-direction is maximal at radius {} with value {}".format(axis, r_c_worst, a_ges_max)
	return a_ges_max
	

def max_dynamic_accel_legacy(robot_config, axis):
	""" Same as max_dynamic_accel but needs more assumptions and is therefore less accurate.
	Calculates the maximal acceleration that can be asserted in a direction given by axis by regarding the maximal centrifugal acceleration when turning. 
	Assumes the center of gravity is in the geometric center and simplifies robot as a 2 wheeled differential driven robot.
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


def write_limits(values):
	"""Saves the compiled limits to a yaml file. The limits will be used from there to determine traversable parts of a map """
	rosparam.print_params(values, new_namespace)

	try:
		# upload to parameter server (needed because rosparam.dump_params only gets values from server)
		rosparam.upload_params(new_namespace, values, verbose=False)
	except rosparam.RosParamIOException:
		print "ERROR: Can't write compiled values to file. Check if roscore is running. You can also set them manually.\n"
		raise rosparam.RosParamIOException
	else:
		# write limits to yaml file
		filename = rospack.get_path(package)+"/robots/jackal_limits.yaml"
		rosparam.dump_params(filename, new_namespace, verbose=False)


def write_limits_occupancy(values):
	"""Saves the compiled limits to a yaml file used by the grid_map_visualization node. The limits will be used as a threshold for an occupancy_grid. """
	content = """grid_map_visualizations:
  - name: traversability_grid
    type: occupancy_grid
    params:
     layer: limits_merged
     data_min: <INCLINE_LIMIT>
     data_max: <INCLINE_LIMIT>"""
	content = content.replace('<INCLINE_LIMIT>', str(values['slope']))
	with open(rospack.get_path(package)+"/config_elevationMap/visualize_raw.yaml", 'w') as f:
		f.write(content)


def write_limits_filter_chain(values):
	"""Saves the compiled limits to a yaml file used by the elevation_mapping and grid_map node. The limits will be used as part of the filter chain that post-processes the elevation map."""
	# reads the template and replaces placeholders ('<EDGE_INCLINE_THRESHOLD>','<SLOPE_LIMIT>','<MAX_SAFE_EDGE_HEIGHT>') with compiled limits
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing_template.yaml", 'r') as f:
		content = f.read()
	
	content = content.replace('<SLOPE_LIMIT>', str(values['slope']))
	content = content.replace('<EDGE_INCLINE_THRESHOLD>', str(values['slope'])) # possibility for extension: could be different than slope
	content = content.replace('<MAX_SAFE_EDGE_HEIGHT>', str(values['edge']))
	content = content.replace('<EDGE_FOOTPRINT>', str(values['footprint_for_edge_determination']))
	
	# save as filter chain
	with open(rospack.get_path(package)+"/config_elevationMap/myGridmapFilters_postprocessing.yaml", 'w') as f:
		f.write(content)


def main_program():
	""" Main function """

	# listeners	
	#sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, get_rotation) # sub_topic = "/odom", sub_msg_type = Odometry
	# publishers
	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

	
	"""
	HOW TO rospy	
	if rospy.has_param('to_delete'):
		global_name = rospy.get_param("/global_name")
		relative_name = rospy.get_param("relative_name")
		private_param = rospy.get_param('~private_name')
		default_param = rospy.get_param('default_param', 'default_value')

		# Using rospy and raw python objects
		rospy.set_param('a_string', 'baz')
		rospy.set_param('~private_int', 2)
		rospy.set_param('list_of_floats', [1., 2., 3., 4.])"""
	
	robot_config = get_robot_config()
	robot_limits = compile2limits(robot_config)
	print "\nCompiled limits:"
	print "incline =", degrees(robot_limits['slope']), 'degrees'
	#write_limits(robot_limits)
	#write_limits_occupancy(robot_limits)
	write_limits_filter_chain(robot_limits)


x, y, z = 'x', 'y', 'z' # z is assumed to point up or downwards
package = 'my_work_pkg'
new_namespace = "/my_namespace_4_jackal_limits"
rospack = rospkg.RosPack()

if __name__ == '__main__':
	print("\nThis utility loads information about the robot from a yaml file, compiles it to limits of what obstacles can be traversed and saves these limits as another yaml file. \nHere we assume the WORST possible robot motion.\n\n")
	try:
		main_program()
	except rospy.ROSInterruptException:
		pass
