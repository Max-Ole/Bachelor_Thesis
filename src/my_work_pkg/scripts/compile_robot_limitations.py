#!/usr/bin/env python2

import rospy
import rosparam
import rospkg
#import sys
from math import sqrt, atan, asin, degrees



def calc_max_inclination(robot_config):
	""" Calculates the maximal inclination for all axes that can safely be traversed with the given geometry and worst possible acceleration and trajectory. """
	accel_max = max(robot_config['acceleration_max'][x], robot_config['acceleration_max'][y], robot_config['acceleration_max'][z])
	maximal_alpha = -float('inf')
	
	for axis in [x,y,z]:
		# project onto current axis ? 
		# static(?) or dynamic(?)
		accel = max(robot_config['acceleration_max'][axis], calc_max_dynamic_accel(robot_config, axis))
		
		# Center of gravity and acceleration
		accel = robot_config['acceleration_max']
		#cog = robot_config['center_of_gravity']
		#wheel = robot_config['wheel']

		l1 = robot_config['dist_cog_wheel'][axis]	# e.g. wheel[x] - cog[x]
		h =  robot_config['dist_cog_wheel'][z]	# cog[z] - wheel[z]   # dist of ground to cog
		a = accel[axis]
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
		
		# keep highest value
		if alpha > maximal_alpha:
			maximal_alpha = alpha
	
	return maximal_alpha


def compile2limits(robot_config):
	"""Compiles information about the robot to determine the limits of what obstacles can be traversed. """
	## inclination max
	angle_limits = []
	
	angle_limits.append(calc_max_inclination(robot_config))
	
	# motor stalling
	angle_limits.append(robot_config['motor_max_incline'])
	
	# slippage due to low friction
	angle_limits.append(robot_config['slip_tolerance_max_incline'])
	
	
	robot_limits = {"incline": min(angle_limits)}
	return robot_limits


def calc_max_dynamic_accel(robot_config, axis):
	""" calculates the maximal acceleration that can be asserted in a direction given by axis by regarding the maximal centrifugal acceleration
v1 := wheel speed outer wheel
v2 := v1*xi wheel speed inner wheel, as factor xi in [-1,1] of other wheel
a = v1^2 / (2*wheels_track) * (1-xi^2) 
Is maximal if v1=v_max and xi=0
==> a_max = v_max^2 / (2*wheels_track) * 1 """
	assert axis in {x,y,z}, "axis must be one of 'x', 'y' or 'z'."
	# get highest v in direction perpendicular to given axes
	v_max = max({robot_config['velocity_max'][xyz] for xyz in {x,y,z} if xyz!=axis})
	
	# distance of wheels in direction of axis
	wheels_track = robot_config['wheels_track'][axis]
	assert wheels_track != 0, "no centrifugal acceleration can be calculated. Wheel track is zero and robot should always tip over."
	
	return v_max**2 / (2*wheels_track)


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


def write_limits_4occupancy(values):
	content = """grid_map_visualizations:
  - name: traversability_grid
    type: occupancy_grid
    params:
     layer: limits_merged
     data_min: <INCLINE_LIMIT>
     data_max: <INCLINE_LIMIT>"""
	content = content.replace('<INCLINE_LIMIT>', str(values['incline']))
	with open(rospack.get_path(package)+"/config_elevationMap/visualize_raw.yaml", 'w') as f:
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
	print "Compiled limits:"
	print "incline =", degrees(robot_limits['incline']), 'degrees'
	#write_limits(robot_limits)
	write_limits_4occupancy(robot_limits)


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
