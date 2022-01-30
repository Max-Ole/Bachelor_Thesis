#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math, time



def get_rotation(msg):
	global roll, pitch, yaw, x, y, z, initial_odom

	position      = msg.pose.pose.position
	orientation_q = msg.pose.pose.orientation

	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	roll_raw, pitch_raw, yaw_raw = euler_from_quaternion(orientation_list)
	x_raw, y_raw, z_raw = [position.x, position.y, position.z]

	if not initial_odom:
		initial_odom = [[x_raw, y_raw, z_raw],[roll_raw, pitch_raw, yaw_raw]]
	else:
		x = x_raw-initial_odom[0][0]
		y = y_raw-initial_odom[0][1]
		z = z_raw-initial_odom[0][2]
		roll  = roll_raw-initial_odom[1][0]
		pitch = pitch_raw-initial_odom[1][1]
		yaw   = yaw_raw-initial_odom[1][2]
	#print "initial_odom", initial_odom
	#print "x, y, z:", x, y, z
	#print "roll, pitch, yaw:", roll, pitch, yaw


def reguar_callback(_):
	# 'climb1meter', 'face_left', 'face_right', 'face_up', 'stop'
	global state

	# start only one initial_odom initialised
	if not initial_odom: return

	if state=='climb1meter':
		done = move_straight(0.8)
		if done:
			state = 'face_left'

	elif state=='face_left':
		done = turn(90)
		if done:
			state = 'face_right'

	elif state=='face_right':
		done = turn(-90)
		if done:
			state = 'face_up'
	elif state=='face_up':
		done = turn(0)
		if done:
			state = 'stop'
	elif state=='stop':
		pub.publish(Twist())
	
	print ""
	print state


def move_straight(target_dist):
	""" Move the specified distance in a straight line """
	command =Twist()
	command.linear.x = kp * (target_dist-x + math.copysign(0.1, target_dist-x))
	pub.publish(command)
	#print("taeget={} current:{}", target_dist,x)
	print  abs(target_dist-x)
	print  abs(target_dist-x) < 1.0/tol
	return abs(target_dist-x) < 1.0/tol


def turn(target_angle):
	""" Turn to the specified target angle """
	#quat = quaternion_from_euler (roll, pitch,yaw)
	#print quat
	command =Twist()
	target_rad = target_angle*math.pi/180
	command.angular.z = kp * (target_rad-yaw + math.copysign(0.2, target_rad-yaw))
	pub.publish(command)
	#print("taeget={} current:{}", target_rad,yaw)
	print  abs(target_rad-yaw)
	print  abs(target_rad-yaw) < math.pi/tol
	return abs(target_rad-yaw) < math.pi/tol


def main_program():
	""" Main function initializes node and subscribers and starts the ROS loop. """
	global sub_odom, pub, initial_odom

	# listeners	
	sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, get_rotation) # sub_topic = "/odom", sub_msg_type = Odometry
	# publishers
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)


tol = 500 # tolerance_divisor
state = 'climb1meter'
roll = pitch = yaw = x = y = z = 0.0
initial_odom = False #[[0,0,0],[0,0,0]]
kp=0.4

if __name__ == '__main__':
	print("This is a test run. Drive up a ramp 1 Meter, turn left, turn right, face back up and stop.")
	time.sleep(5)
	try:
		rospy.init_node('testrun_ramp_1m_turn')
		main_program()
		rospy.Timer(rospy.Duration(0.1), reguar_callback) # Set callback
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

""" RESET odometry position to 0 before starting test! Use following service call to reset  position. Orientation needs to be set seperatly!
rosservice call /set_pose "pose:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  pose:
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" """
