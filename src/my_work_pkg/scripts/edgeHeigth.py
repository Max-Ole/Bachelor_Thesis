#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry # type: nav_msgs/Odometry
from grid_map_msgs.msg import GridMap # type: grid_map_msgs/GridMap
import numpy as np



def grid_map_callback(msg):
	""" called trough subsriber """

	print("type:", type(msg))
	print("dir:", dir(msg))
	print(msg.layers)
	print(msg.basic_layers)
	print(type(msg.data))

#	# Line Iterator:
#	pt_a = np.array([10, 11]) # start punkt
#       pt_b = np.array([45, 67]) # end punkt
#       im = np.zeros((80, 80, 3), np.uint8) # daten
#       for p in np.linspace(pt_a, pt_b, np.linalg.norm(pt_a-pt_b)):
            # do stuff with p
            # access grid map like this:
            # https://docs.ros.org/en/kinetic/api/grid_map_msgs/html/msg/GridMap.html


def reguar_callback(msg):
	""" Gets called regulary from rospy.Timer """
	pass


def main_program():
	""" Main function initializes node and subscribers and starts
		the ROS loop. """
	global sub_odom, sub_elevation, pub, pub_elevation, pub_elevation_raw

	print("Receives an elevation_mapping Grid_map message and calculates the height of edges from the edge and elevation layer.")
	# listeners	
	#sub_odom = rospy.Subscriber("/odom", Odometry, sub_odom_callback) # sub_topic = "/odom", sub_msg_type = Odometry
	sub_elevation_raw = rospy.Subscriber("/elevation_mapping/elevation_map_raw", GridMap, grid_map_callback)

	# publishers
	#pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
	pub_elevation_raw = rospy.Publisher("/elevation_mapping/elevation_map_raw", GridMap, queue_size=10)



if __name__ == '__main__':
	try:
		rospy.init_node('filterChain_edgeHeight')
		main_program()
		#rospy.Timer(rospy.Duration(0.5), reguar_callback) # Set callback
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
