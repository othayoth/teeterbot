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
import tf 
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import tf2_geometry_msgs
import numpy 
from  sensor_msgs.msg import Imu

class robot_controller():
    def __init__(self):


        self.got_new_msg = False        
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
        
        self.tilt_old = 0.0
        self.tilt_current = 0.0        
        self.dtilt = 0.0
        self.dt = 0.001
        
        

        # State feedback gain1s from LQR
        self.K_r = [  -424.2816, -391.0335,  -209.2573,   424.2816]
        self.K_l = [  -424.2816, -391.0335,  -209.2573,  -424.2816]
        self.g_r = [ -424.2816,    424.2816]
        self.g_l = [ -424.2816,   -424.2816]

        self.K_r = [  -23.3150, -100.5295,  -33.9993,    7.3668,   15.8114,  -15.8114]
        self.K_l = [  -23.3150, -100.5295,  -33.9993,   -7.3668,   15.8114,   15.8114]

        # control torques for right and left wheels
        self.control_effort = [0.0,0.0]
        
        # body orientations
        self.euler = [0.0,0.0,0.0]
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        
        # create publishers and subscribers
        self.sub_model       = rospy.Subscriber("gazebo/link_states", ModelStates, self.state_feedback_controller)
        self.sub_reference   = rospy.Subscriber("/cmd_vel", Twist, self.reference_callback)
        self.sub_fallen      = rospy.Subscriber("teeterbot/fallen_over", Bool, self.reset_controller)        
        self.sub_right_speed = rospy.Subscriber("teeterbot/right_wheel_speed", Float64, self.right_wheel_speed_callback)        
        self.sub_left_speed  = rospy.Subscriber("teeterbot/left_wheel_speed", Float64, self.left_wheel_speed_callback)        
        self.sub_imu         = rospy.Subscriber("teeterbot/imu", Imu, self.imu_callback)        
        self.pub_right_wheel = rospy.Publisher( "teeterbot/right_torque_cmd", Float64,queue_size=10)
        self.pub_left_wheel  = rospy.Publisher( "teeterbot/left_torque_cmd", Float64,queue_size=10)
        
        self.tf_listener = tf.TransformListener()

        # Main while loop.
        while not rospy.is_shutdown():                        
            if self.upright:
                self.pub_right_wheel.publish(self.control_effort[0])
                self.pub_left_wheel.publish(self.control_effort[1])                
                #print("published torque is "+ str(self.control_effort[0]) + ", " + str(self.control_effort[1]) )
            else:
                self.pub_right_wheel.publish(0.0)
                self.pub_left_wheel.publish(0.0)
            
                
    
    # reset if fallen over
    def reset_controller(self,msg):            
        if(msg.data):
            print("teeterbot fell over; setting control effort to 0.0")            
            self.control_effort = [0.0,0.0]
            self.pub_right_wheel.publish(0.0)
            self.pub_left_wheel.publish(0.0)  
            #time.sleep(4.0)
                      
            self.upright = False
        else:
            print("teeterbot upright; begin control")            
            self.upright = True
            
    
    # update references as the commands come in
    def reference_callback(self,msg):                
        self.ref_forward_speed = msg.linear.x
        self.ref_turning_speed = msg.angular.z       
        print("Updated reference")

    def imu_callback(self,msg):
        self.dtilt = msg.angular_velocity.y

    def right_wheel_speed_callback(self,msg):                
        self.rw_speed = msg.data
    def left_wheel_speed_callback(self,msg):                
        self.lw_speed = msg.data
        


    def state_feedback_controller(self,msg):
        
        # convert orientation to roll pitch yaw
        quat = (msg.pose[1].orientation.x,
                msg.pose[1].orientation.y,
                msg.pose[1].orientation.z,
                msg.pose[1].orientation.w)
        self.euler = tf.transformations.euler_from_quaternion(quat)                
        self.tilt_current = self.euler[1]
        #self.dtilt  = (self.tilt_current - self.tilt_old)/self.dt
        #self.tilt_old = self.tilt_current
               

        (trans,rot) = self.tf_listener.lookupTransform("/world", "/base_link", rospy.Time(0))
        rot_inv = tf.transformations.quaternion_inverse(rot)
        
        rot_mat = tf.transformations.quaternion_matrix(rot)
        rot_mat = (rot_mat[0:3,0:3]).T

        v = [0.0, 0.0, 0.0]
        v[0] = msg.twist[1].linear.x
        v[1] = msg.twist[1].linear.y
        v[2] = msg.twist[1].linear.z
        
        v = rot_mat.dot(v)
        # print(v)

        #vt = tf2_geometry_msgs.do_transform_vector3(v, rot_inv)
        
    
        # observed states
        self.x[0] =( msg.twist[1].linear.x*math.cos(self.euler[2]) +        # forward velocity
                     msg.twist[1].linear.y*math.sin(self.euler[2]))
        #self.x[0] = v[0]
        #self.x[0] = 0.2*(self.lw_speed + self.rw_speed)/2       # forward velocity
        self.x[1] = self.euler[1]                  # body tilt
        self.x[2] = msg.twist[1].angular.y      # body tilt velocity
        #self.x[2] =-self.dtilt      # body tilt velocity
        self.x[2] =(-msg.twist[1].angular.x*math.sin(self.euler[2]) +        # forward velocity
                     msg.twist[1].angular.y*math.cos(self.euler[2]))
        self.x[3] = msg.twist[1].angular.z      # turning velocity
        self.x[3] =0.2*(self.rw_speed - self.lw_speed)/(2*0.5)       # forward velocity

        print(self.x)
        # control effort u = -K*x for state feedback controller
        # self.control_effort[0] = -(self.K_r[0]*self.x[0] + 
        #                            self.K_r[1]*self.x[1] +
        #                            self.K_r[2]*self.x[2] +
        #                            self.K_r[3]*self.x[3])
        # self.control_effort[0] = (self.control_effort[0] + 
        #                           self.g_r[0]*self.ref_forward_speed +                                
        #                           self.g_r[1]*self.ref_turning_speed)                                   
        # self.control_effort[1] = -(self.K_l[0]*self.x[0] + 
        #                            self.K_l[1]*self.x[1] +
        #                            self.K_l[2]*self.x[2] +
        #                            self.K_l[3]*self.x[3])                                  
        # self.control_effort[1] = (self.control_effort[1] + 
        #                           self.g_l[0]*self.ref_forward_speed + 
        #                           self.g_l[1]*self.ref_turning_speed)                                   
        self.control_effort[0] = -(self.K_r[0]*self.x[0] + 
                                   self.K_r[1]*self.x[1] +
                                   self.K_r[2]*self.x[2] +
                                   self.K_r[3]*self.x[3] +        
                                   self.K_r[4]*(self.ref_forward_speed-self.x[0]) +                                
                                   self.K_r[5]*(self.ref_turning_speed-self.x[3]))                                   
        self.control_effort[1] = -(self.K_l[0]*self.x[0] + 
                                   self.K_l[1]*self.x[1] +
                                   self.K_l[2]*self.x[2] +
                                   self.K_l[3]*self.x[3] +        
                                   self.K_l[4]*(self.ref_forward_speed-self.x[0]) +                                
                                   self.K_l[5]*(self.ref_turning_speed-self.x[3]))                                                                      
        

        #self.control_effort=[0.0, 0.0]

        # cutoff control if robot rolls or pitches too much
        if abs(self.roll*180/math.pi) > 5 or abs(self.pitch*180/math.pi)>45:
            self.control_effort=[0.0, 0.0]

        
        #print("Torques: " + str(self.control_effort[0]) + ", " + str(self.control_effort[1]))

        
        self.got_new_msg  = True

        

    
if __name__ == '__main__':
    rospy.init_node('robot_controller')
    try:
        robot_controller()
        robot_controller.tf_listener = tf.TransformListener()
    except rospy.ROSInterruptException: pass