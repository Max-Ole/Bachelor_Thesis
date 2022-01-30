#!/usr/bin/env python

import rospy
from grid_map_msgs.msg import GridMap # type: grid_map_msgs/GridMap



def sub_elevation_callback(msg):
	""" called trough subscriber """
	global recodedMsg1
	#print("type:", type(msg))
	#print("dir:", dir(msg))
	if not recodedMsg1:
		print("\nReceived fused elevation map\n")
		recodedMsg1 = msg

def sub_elevation_raw_callback(msg):
	""" called trough subscriber """
	global recodedMsg2
	#print("type:", type(msg))
	#print("dir:", dir(msg))
	if not recodedMsg2:
		print("\nReceived a RAW elevation map\n")
		recodedMsg2 = msg


def reguar_callback(msg):
	""" Gets called regularly from rospy.Timer """
	global recodedMsg1, pub_elevation, recodedMsg2, pub_elevation_raw
	if recodedMsg1:
		pub_elevation.publish(recodedMsg1)
		print("pub fused")
	if recodedMsg2:
		pub_elevation_raw.publish(recodedMsg2)
		print("pub raw")


def main_program():
	""" Main function initializes node and subscribers and starts
		the ROS loop. """
	global sub_odom, sub_elevation, pub, pub_elevation, pub_elevation_raw

	print("This tool reads a single elevation_mapping Grid_map message (raw or fused) and repeats it indefinitely.\nThis is to display a bag file with a single message in rviz. Run this script and then play the rosbag. The single repeated and rviz can pick it up.")

	sub_elevation = rospy.Subscriber("/elevation_mapping/elevation_map", GridMap, sub_elevation_callback)
	sub_elevation_raw = rospy.Subscriber("/elevation_mapping/elevation_map_raw", GridMap, sub_elevation_raw_callback)

	# publishers
	#pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
	pub_elevation = rospy.Publisher("/elevation_mapping/elevation_map", GridMap, queue_size=10)
	pub_elevation_raw = rospy.Publisher("/elevation_mapping/elevation_map_raw", GridMap, queue_size=10)


recodedMsg1 = False
recodedMsg2 = False

if __name__ == '__main__':
	try:
		rospy.init_node('grid_map_repeater')
		main_program()
		rospy.Timer(rospy.Duration(0.5), reguar_callback) # Set callback
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
