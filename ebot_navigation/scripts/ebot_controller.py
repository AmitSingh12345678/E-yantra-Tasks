#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
#   use rospy.spin()
if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        waypoints=[[-9.1,-1.2],[10.7,10.5],[12.6,-1.9],[18.2,-1.4],[-2,4]]
        for i in range(0,5) :
           result = movebase_client(waypoints[i][0],waypoints[i][1])
           if result:
            rospy.loginfo("Goal execution done!")
            rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
