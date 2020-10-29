#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pose=[0,0,0]
regions=[]
INF=100000000
P=10
VELOCITY=1
FRONT_DIST=0.7
DIST=1.5
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
        x=pose[0]+FRONT_DIST
        y=2*math.sin(x)*math.sin(x/2)
        if x>6.28:
            velocity_msg.linear.x=0
            velocity_msg.angular.z=0
            pub.publish(velocity_msg)
            break

        theta_goal=math.atan((y-pose[1])/(FRONT_DIST))
        theta=theta_goal-pose[2]
        w=P*theta
        velocity_msg.linear.x = VELOCITY
        velocity_msg.angular.z = w
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

def avoid_obstacle():
    start_x = 6.28
    start_y = 0
    goal_line_slope = (GOAL_Y-start_y)/(GOAL_X-start_x)
    print("Angle to goal: ")
    print(math.atan(goal_line_slope))
    c = -2*math.pi*goal_line_slope
    HIT_X=0    
    hit_x_found=False
    while not rospy.is_shutdown():

        if abs(pose[1]-goal_line_slope*pose[0]-c)<0.1 and HIT_X!=0 and pose[0]>HIT_X:
              velocity_msg.linear.x=0
              velocity_msg.angular.z=1
              pub.publish(velocity_msg)
              print("Stop Location:-")
              print(pose[0])
              print(pose[1])
              break
        if regions['front']<=DIST:
            if not hit_x_found:
               HIT_X=pose[0]
               hit_x_found=True
            velocity_msg.linear.x=0
            velocity_msg.angular.z=1
        else:
            velocity_msg.linear.x=0.6
            velocity_msg.angular.z=0
            
        pub.publish(velocity_msg)       
        if regions['front']>=INF:
            if regions['fright']>=DIST/2:
                velocity_msg.angular.z=-1
            elif regions['fright']<=DIST/4:
                velocity_msg.angular.z=1       
            else:
                velocity_msg.angular.z=0

            velocity_msg.linear.x=0.6
            pub.publish(velocity_msg)    
        rate.sleep()

def line_found():
    while True:
        if pose[2]-math.atan(goal_line_slope)>=-0.01 and pose[2]-math.atan(goal_line_slope)<=0.01:
           velocity_msg.linear.x=0
           velocity_msg.angular.z=0
           pub.publish(velocity_msg)
           break
    velocity_msg.linear.x=1
    velocity_msg.angular.z=0
    pub.publish(velocity_msg)

def control_loop():
    rospy.init_node('ebot_controller')

    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    global rate
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    ####TRACE PATH####
    trace_path()
    
    ####AVOID OBST####
    avoid_obstacle()

    ####LINE FOUND####    
    line_found()
    
    #######AFTER REACH#########
    
    while True:
        if pose[0]>=GOAL_X:
            velocity_msg.linear.x=0
            velocity_msg.angular.z=0
            print("Reached Goal:-")
            print(pose[0])
            print(pose[1])
            print(pose[2])
            pub.publish(velocity_msg)
            break

    rospy.spin()
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
