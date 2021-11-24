#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def twist_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo("twist callback reached")
    
    
def robot_controller():

    rospy.init_node('robot_controller', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, twist_callback)
    
    rospy.loginfo("robot_controller active")
    

    rospy.spin()

if __name__ == '__main__':
    robot_controller()