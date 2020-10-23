#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

TURTLE_LINEAR_VEL = 2.0
TURTLE_ANGULAR_VEL = 1.0
turtle_has_returned = False

def record_turtle_pos(turtle_pos_msg):
   
  theta = turtle_pos_msg.theta
# if (theta < 0),then we should add 2*pi
  if theta < 0:    
    theta += 2*(math.pi) 
    
  global turtle_has_returned    
  if not turtle_has_returned:
    rospy.loginfo("Moving in a circle")
    print(theta)
# if theta equals to 2*pi,then it means turtle reaches back to it's initial pos.
  if theta >= 6.27 and theta <= 6.29: 
    turtle_has_returned = True
    

def main():
    
  rospy.init_node("node_turtle_revolve",anonymous = True)
  
  turtle_vel_pub = rospy.Publisher("turtle1/cmd_vel",Twist,queue_size = 10)
  
  rospy.Subscriber("/turtle1/pose",Pose,record_turtle_pos)
  
  loop_rate = rospy.Rate(10)
  
  msg_obj = Twist() 
  
  while not rospy.is_shutdown():
      
    global turtle_has_returned
#   if turtle_has_returned == True,then turtle should stop
    if turtle_has_returned:  
        msg_obj.linear.x = 0
        msg_obj.angular.z = 0
        turtle_vel_pub.publish(msg_obj)
        rospy.loginfo("GOAL REACHED!")
    	break
#   else turtle will move with some linear and angular velocity        
    msg_obj.linear.x = TURTLE_LINEAR_VEL
    msg_obj.angular.z = TURTLE_ANGULAR_VEL
    turtle_vel_pub.publish(msg_obj)
    
    loop_rate.sleep()

  rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
