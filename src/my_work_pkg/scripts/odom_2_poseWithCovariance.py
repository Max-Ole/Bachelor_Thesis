#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def extractPose(msg):
	global pub
	
	pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time(0)
    #pose.header.frame_id = from_frame
    pose.pose = msg.pose
	
	pub.publish(pose)


def main_program():
	""" Main function initializes node and subscribers and starts the ROS loop. """
	global sub_odom, pub

	print("This is a simple conversion tool that reads odometry and calculates yaw pitch roll angles from the quaternions.")
	# listeners	
	sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, extractPose)
	pub = rospy.Publisher("pose_from_odomFiltered", PoseWithCovarianceStamped, queue_size=10)



if __name__ == '__main__':
	try:
		rospy.init_node('odom_2_poseWithCov')
		main_program()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
