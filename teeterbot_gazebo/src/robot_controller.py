#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from dynamic_reconfigure.server import Server
from teeterbot_gazebo.cfg import PIDConfig
import sys
import time


class robot_controller():
    def __init__(self):


        self.got_new_msg = False
        self.on = True
        self.upright = True

        self.x = [0.0, 0.0, 0.0, 0.0] # state = [forward velocity, body tilt, body tilt velocity, turning velocity]
        self.state = Twist()
        self.reference = Twist()
        self.error = Twist()
        self.ref_forward_speed = 0.0  # reference forward speed specified by the user        
        self.ref_turning_speed = 0.0  # reference turning speed specified by the user
        self.ref_tilt          = 0.0  # reference tilt that is desired
        self.ref_tilt_speed    = 0.0  # reference tilt speed that is desired

        # right and left wheel speeds
        self.rw_speed = 0.0
        self.lw_speed = 0.0

        # State feedback gains from LQR
        self.K_r = [ -2.2361,-19.3436,-2.8267, 2.23611]
        self.K_l  = [ -2.2361,-19.3436,-2.8267,-2.23611]

        # self.g_r = [ -2.2361, 2.2361]
        # self.g_l  = [ -2.2361,-2.2361]

        # self.g_r = [1, 1]
        # self.g_l  = [1,-1]

        # State feedback gain1s from LQR
        self.K_r = [ -2.2361,-20.6334,-4.0161, 2.23611]
        self.K_l = [ -2.2361,-20.6334,-4.0161,-2.23611]
        self.g_r = [ -2.2361,    2.2361]
        self.g_l = [ -2.2361,   -2.2361]

        # control torques for right and left wheels
        self.control_effort = [0.0,0.0]
        
        # body orientations
        self.euler = [0.0,0.0,0.0]
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        
        # create publishers and subscribers
        sub_model       = rospy.Subscriber("gazebo/model_states", ModelStates, self.state_feedback_controller)
        sub_reference   = rospy.Subscriber("/cmd_vel", Twist, self.reference_callback)
        sub_fallen      = rospy.Subscriber("teeterbot/fallen_over", Bool, self.reset_controller)
        self.sub_rw_speed = rospy.Subscriber("teeterbot/right_wheel_speed", Float64 , self.rw_speed_callback)
        self.sub_lw_speed = rospy.Subscriber("teeterbot/left_wheel_speed", Float64 , self.lw_speed_callback)
        self.pub_right_wheel = rospy.Publisher("teeterbot/right_torque_cmd", Float64,queue_size=1)
        self.pub_left_wheel  = rospy.Publisher("teeterbot/left_torque_cmd", Float64,queue_size=1)
        self.pub_robot_state = rospy.Publisher("teeterbot/state", Twist, queue_size=1)
        self.pub_robot_ref   = rospy.Publisher("teeterbot/reference", Twist, queue_size=1)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg and self.upright:
                if self.on and self.upright:
                    self.pub_right_wheel.publish(self.control_effort[0])
                    self.pub_left_wheel.publish(self.control_effort[1])
                    self.pub_robot_ref.publish(self.reference)
                    self.pub_robot_state.publish(self.state)
                else:
                    self.pub_right_wheel.publish(0.0)
                    self.pub_left_wheel.publish(0.0)
                self.got_new_msg = False

    def lw_speed_callback(self,msg):
        self.lw_speed = msg.data
    def rw_speed_callback(self,msg):
        self.lw_speed = msg.data

    # reset if fallen over
    def reset_controller(self,msg):        
        print("in reset callback")
        if(msg.data):
            print("teeterbot fell over")            
            self.control_effort = [0.0,0.0]
            self.pub_right_wheel.publish(0.0)
            self.pub_left_wheel.publish(0.0)
            time.sleep(5.0)
            self.upright = True
            
    
    # update references as the commands come in
    def reference_callback(self,msg):                
        self.ref_forward_speed = msg.linear.x
        self.ref_turning_speed = msg.angular.z
        self.reference.linear.x  = self.ref_forward_speed 
        self.reference.angular.x = self.ref_tilt
        self.reference.angular.y = self.ref_tilt_speed
        self.reference.angular.z = self.ref_turning_speed
        self.pub_robot_state.publish(self.reference)
        print("Updated reference")

    def state_feedback_controller(self,msg):
        
        # convert orientation to roll pitch yaw
        quat = (msg.pose[1].orientation.x,
                msg.pose[1].orientation.y,
                msg.pose[1].orientation.z,
                msg.pose[1].orientation.w)
        self.euler = tf.transformations.euler_from_quaternion(quat)
        self.roll  = self.euler[0]
        self.pitch = self.euler[1]
        self.yaw   = self.euler[2]
    
        # observed states
        self.x[0] = msg.twist[1].linear.x       # forward velocity
        #self.x[0] = 0.2*(self.lw_speed + self.rw_speed)/2       # forward velocity
        self.x[1] = self.pitch                  # body tilt
        self.x[2] = msg.twist[1].angular.y      # body tilt velocity
        self.x[3] = msg.twist[1].angular.z      # turning velocity
        #self.x[3] = 0.2*(self.rw_speed - self.rw_speed)/(2*0.5)       # forward velocity

        self.state.linear.x  = self.x[0]
        self.state.angular.x = self.x[1]
        self.state.angular.y = self.x[2]
        self.state.angular.z = self.x[3]

        # control effort u = -K*x for state feedback controller
        self.control_effort[0] = -(self.K_r[0]*self.x[0] + 
                                   self.K_r[1]*self.x[1] +
                                   self.K_r[2]*self.x[2] +
                                   self.K_r[3]*self.x[3])
        self.control_effort[0] = (self.control_effort[0] + 
                                  self.g_r[0]*self.ref_forward_speed +                                
                                  self.g_r[1]*self.ref_turning_speed)                                   
        self.control_effort[1] = -(self.K_l[0]*self.x[0] + 
                                   self.K_l[1]*self.x[1] +
                                   self.K_l[2]*self.x[2] +
                                   self.K_l[3]*self.x[3])                                  
        self.control_effort[1] = (self.control_effort[1] + 
                                  self.g_l[0]*self.ref_forward_speed + 
                                  self.g_l[1]*self.ref_turning_speed)                                   

        # cutoff control if robot rolls or pitches too much
        if abs(self.roll*180/math.pi) > 5 or abs(self.pitch*180/math.pi)>75:
            self.control_effort=[0.0, 0.0]

        
        #print("Torques: " + str(self.control_effort[0]) + ", " + str(self.control_effort[1]))

        
        self.got_new_msg  = True

        

    
if __name__ == '__main__':
    rospy.init_node('robot_controller')
    try:
        robot_controller()
    except rospy.ROSInterruptException: pass