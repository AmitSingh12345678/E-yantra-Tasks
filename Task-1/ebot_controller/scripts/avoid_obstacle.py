#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pose=[0,0,0]
regions=[]
INFINITY=100000000
P=10
PATH_COURSE_VELOCITY=1
OBSTACLE_COURSE_VELOCITY=0.6
GOAL_REACH_VELOCITY=1
LOOK_AHEAD_DIST=0.7
AVOID_DIST=1.5
MAX_SIDE_DIST = AVOID_DIST/2
MIN_SIDE_DIST = AVOID_DIST/4
THETA_TOLERANCE=0.1
GOAL_X=12.5
GOAL_Y=0

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def laser_callback(msg):
    global regions
    regions = {
        'bright':   min(msg.ranges[0:143]),
        'fright':   min(msg.ranges[144:287]),
        'front':    min(msg.ranges[288:431]),
        'fleft':    min(msg.ranges[432:575]),
        'bleft':    min(msg.ranges[575:720])  ,
    }

def trace_path():
    while not rospy.is_shutdown():
        global pose

        x=pose[0]+LOOK_AHEAD_DIST
        y=2*math.sin(x)*math.sin(x/2)
        if x>2*math.pi:
            velocity_msg.linear.x=0
            velocity_msg.angular.z=0
            pub.publish(velocity_msg)
            break

        theta_goal=math.atan((y-pose[1])/(LOOK_AHEAD_DIST))
        theta_deviation=theta_goal-pose[2]
        w=P*theta_deviation
        velocity_msg.linear.x = PATH_COURSE_VELOCITY
        velocity_msg.angular.z = w
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

def avoid_obstacle():
    while not rospy.is_shutdown():

        theta_goal=math.atan((GOAL_Y-pose[1])/(GOAL_X-pose[0]))
        theta_bot=pose[2]
        
        #Path to goal is clear
        if abs(theta_goal-theta_bot) <=THETA_TOLERANCE and regions['front'] > INFINITY:
            velocity_msg.linear.x=GOAL_REACH_VELOCITY
            velocity_msg.angular.z=0
            pub.publish(velocity_msg)
            break

        if regions['front']<=AVOID_DIST:
            velocity_msg.linear.x=0
            velocity_msg.angular.z=1
        else:
            velocity_msg.linear.x=OBSTACLE_COURSE_VELOCITY
            velocity_msg.angular.z=0
            
        pub.publish(velocity_msg)       

        if regions['front']>=INFINITY:
            if regions['fright']>=MAX_SIDE_DIST:
                velocity_msg.angular.z=-1
            elif regions['fright']<=MIN_SIDE_DIST:
                velocity_msg.angular.z=1       
            else:
                velocity_msg.angular.z=0

            velocity_msg.linear.x=OBSTACLE_COURSE_VELOCITY
            pub.publish(velocity_msg)   

        rate.sleep()

def control_loop():
    rospy.init_node('ebot_controller')
    
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    global rate
    rate = rospy.Rate(10) 

    global velocity_msg
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    
    trace_path()
    avoid_obstacle()
        
    while True:
        if pose[0]>=GOAL_X:
            velocity_msg.linear.x=0
            velocity_msg.angular.z=0
            pub.publish(velocity_msg)
            print("Reached Goal!")
            print("Current Coordinates:-")
            print(pose[0])
            print(pose[1])
            break

    rospy.spin()
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass