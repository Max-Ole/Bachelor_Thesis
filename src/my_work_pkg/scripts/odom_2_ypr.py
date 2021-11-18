#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def get_rotation(msg):

	position      = msg.pose.pose.position
	orientation_q = msg.pose.pose.orientation

	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	roll, pitch, yaw = euler_from_quaternion(orientation_list)
	x, y, z = [position.x, position.y, position.z]

	print "\nx:\t", round(x, 3)
	print "y:\t", round(y, 3)
	print "z:\t", round(z, 3)
	print "roll:\t", round(roll, 3)
	print "pitch:\t", round(pitch, 3)
	print "yaw:\t", round(yaw, 3)


def main_program():
	""" Main function initializes node and subscribers and starts the ROS loop. """
	global sub_odom, pub, initial_odom

	print("This is a simple conversion tool that reads odometry and calculates yaw pitch roll angles from the quaternions.")
	# listeners	
	sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, get_rotation)


if __name__ == '__main__':
	try:
		rospy.init_node('odom_2_ypr_displayer')
		main_program()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
