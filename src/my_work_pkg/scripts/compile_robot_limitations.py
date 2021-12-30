#!/usr/bin/env python2

import rospy
import rosparam
import rospkg
from math import sqrt, atan, asin, degrees



def compile2limits(robot_config):
	"""Compiles information about the robot to determine the limits of what obstacles can be traversed. """
	## inclination max
	angle_limits = {'tipping over': float('inf'), \
					'motor stalling': float('inf'), \
					'slippage': float('inf') \
					}
	# tipping over
	angle_limits['tipping over'] = max_incline_tipping(robot_config)
	
	# motor stalling
	angle_limits['motor stalling'] = max_incline_torque(robot_config) #robot_config['motor_max_incline']
	
	# slippage due to low friction
	angle_limits['slippage'] = robot_config['slip_tolerance_max_incline']
	
	## edge max
	edge_limits = {'ground clearance': float('inf')}
	# ground clearance
	edge_limits['ground clearance'] = robot_config['ground_clearance']
	# TODO add front or back of robot hitting ramp
	
	# return limit and output the reason for that limit aka. the most restrictive design choice for traversability.
	reason = min(angle_limits, key=lambda k: angle_limits[k])
	print "\nThe threat of {} ist the most restrictive for the highest safely traversable inclination.\n".format(reason)
	robot_limits = {'slope': angle_limits[reason], 'edge':edge_limits['ground clearance']}
	return robot_limits


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
	content = content.replace('<EDGE_INCLINE_THRESHOLD>', str(values['slope'])) # TODO could be different than slope?
	content = content.replace('<MAX_SAFE_EDGE_HEIGHT>', str(values['edge']))
	
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

if __name__ == '__main__':
	print("\nThis utility loads information about the robot from a yaml file, compiles it to limits of what obstacles can be traversed and saves these limits as another yaml file.\n")
	try:
		rospack = rospkg.RosPack()
		main_program()
	except rospy.ROSInterruptException:
		pass
