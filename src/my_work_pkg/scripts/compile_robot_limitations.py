#!/usr/bin/env python2

import rospy
import rosparam
import rospkg
#import sys
from math import sqrt, atan, asin, degrees



def compile2limits(robot_config):
	"""Compiles information about the robot to determine the limits of what obstacles can be traversed. """
	## inclination max
	angle_limits = []

	# Center of gravity and acceleration
	accel = robot_config['acceleration_max']
	#cog = robot_config['center_of_gravity']
	#wheel = robot_config['wheel']

	# TODO loop all axes x,y,z

	l1 = robot_config['dist_cog_wheel'][x]	# wheel[x] - cog[x]
	h =  robot_config['dist_cog_wheel'][z]	# cog[z] - wheel[z]   # dist of ground to cog
	a = accel[x]
	g = 9.81
	b = sqrt(h**2 + l1**2)
	gamma = atan(l1/h)

	try:
		alpha1 = gamma - asin( (a*h)/(g*b) )
	except ValueError: # accelerating at max acceleration would tip robot regardless of inclination
		print "ERROR: Robot seems inherently unstable. Please check the robot property values. E.g. the max acceleration could be too high.\n"
		raise ValueError
	else:
		if alpha1<0.0: # max acceleration only doesn't tip robot if accelerating downhill
			print "ERROR: Robot seems inherently unstable. Please check the robot property values. E.g. the max acceleration could be too high.\n"
			raise ValueError
	angle_limits.append(alpha1)

	# motor stalling
	angle_limits.append(robot_config['motor_max_incline'])

	# slipage due to low friction
	angle_limits.append(robot_config['slip_tolerance_max_incline'])


	robot_limits = {"incline": min(angle_limits)}
	return robot_limits


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
     layer: slope
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


x, y, z = 'x', 'y', 'z'
package = 'my_work_pkg'
new_namespace = "/my_namespace_4_jackal_limits"

if __name__ == '__main__':
	print("\nThis utility loads information about the robot from a yaml file, compiles it to limits of what obstacles can be traversed and saves these limits as another yaml file.\n")
	try:
		#rospy.init_node('testrun_ramp_1m_turn')
		rospack = rospkg.RosPack()
		main_program()
		#rospy.Timer(rospy.Duration(0.1), reguar_callback) # Set callback
		#rospy.spin()
	except rospy.ROSInterruptException:
		pass
