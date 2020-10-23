#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pose=[0,0,0]

def odom_callback(data):
	global pose
	x  = data.pose.pose.orientation.x;
	y  = data.pose.pose.orientation.y;
	z = data.pose.pose.orientation.z;
	w = data.pose.pose.orientation.w;
	pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

# def laser_callback(msg):
#     global regions
#     regions = {
#         'bright':  	,
#         'fright': 	,
#         'front':  	,
#         'fleft':  	,
#         'bleft':   	,
#     }

def control_loop():
	rospy.init_node('ebot_controller')
	
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	# rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
	rospy.Subscriber('/odom', Odometry, odom_callback)
	
	rate = rospy.Rate(10) 

	velocity_msg = Twist()
	velocity_msg.linear.x = 0
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)
	
	while not rospy.is_shutdown():

		global pose
		x=pose[0]+0.7
		y=2*math.sin(x)*math.sin(x/2)
		if x>6.28+0.7:
			velocity_msg.linear.x=0
			velocity_msg.angular.z=0
			pub.publish(velocity_msg)
			break

		theta_goal=math.atan((y-pose[1])/(0.7))
		print("theta_goal is ",theta_goal)
		theta=theta_goal-pose[2]
		print("theta is ",theta)
		P=10
		v=1
		w=P*theta
		velocity_msg.linear.x = v
		velocity_msg.angular.z = w
		pub.publish(velocity_msg)
		print("Controller message pushed at {}".format(rospy.get_time()))
		rate.sleep()

	# while True:
	# 	if pose[2]>=-0.1 and pose[2]<=0.1:
	# 	   velocity_msg.linear.x=0
	# 	   velocity_msg.angular.z=0
	# 	   pub.publish(velocity_msg)
	# 	   break

	rospy.spin()
if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass
