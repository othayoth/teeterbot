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


class robot_controller():
    def __init__(self):

        self.srv = Server(PIDConfig, self.config_callback)
        self.got_new_msg = False
        self.on = True
        self.upright = True

        self.x = [0.0, 0.0, 0.0, 0.0] # state = [forward velocity, body tilt, body tilt velocity, turning velocity]
        
        self.ref_forward_speed = 0.0  # reference forward speed specified by the user        
        self.ref_turning_speed = 0.0  # reference turning speed specified by the user


        # State feedback gains from LQR
        self.K_lqr_right = [ -2.2361,-19.3436,-2.8267, 2.23611]
        self.K_lqr_left  = [ -2.2361,-19.3436,-2.8267,-2.23611]

        self.g_lqr_right = [ -2.2361, 2.2361]
        self.g_lqr_left  = [ -2.2361,-2.2361]

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
        pub_right_wheel = rospy.Publisher("teeterbot/right_torque_cmd", Float64)
        pub_left_wheel  = rospy.Publisher("teeterbot/left_torque_cmd", Float64)
        #pub_robot_state = rospy.Publisher("teeterbot/state", Twist, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg and self.upright:
                if self.on:
                    pub_right_wheel.publish(self.control_effort[0])
                    pub_left_wheel.publish(self.control_effort[1])
                else:
                    pub_right_wheel.publish(0.0)
                    pub_left_wheel.publish(0.0)
                self.got_new_msg = False

    def reset_controller(self,msg):        
        if(msg.data):
            print("teeterbot fell over")
            self.Eold = 0
            self.E = 0
            self.control_effort = [0.0,0.0]
            self.upright = False
            self.got_new_msg  = True
        else:
            self.upright = True
            self.got_new_msg  = True
    
    # update references as the commands come in
    def reference_callback(self,msg):
        self.ref_forward_speed = msg.linear.x
        self.ref_turning_speed = msg.angular.z

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
        self.x[1] = self.pitch                  # body tilt
        self.x[2] = msg.twist[1].angular.y      # body tilt velocity
        self.x[3] = msg.twist[1].angular.z      # turning velocity

        # control effort u = -K*x for state feedback controller
        self.control_effort[0] = -(self.K_lqr_right[0]*self.x[0] + 
                                   self.K_lqr_right[1]*self.x[1] +
                                   self.K_lqr_right[2]*self.x[2] +
                                   self.K_lqr_right[3]*self.x[3])
        self.control_effort[0] = self.control_effort[0] + self.g_lqr_right[0]*self.ref_forward_speed + self.g_lqr_right[1]*self.ref_turning_speed
        self.control_effort[1] = -(self.K_lqr_left[0]*self.x[0] + 
                                   self.K_lqr_left[1]*self.x[1] +
                                   self.K_lqr_left[2]*self.x[2] +
                                   self.K_lqr_left[3]*self.x[3])                                  
        self.control_effort[1] = self.control_effort[1] + self.g_lqr_left[0]*self.ref_forward_speed + self.g_lqr_left[1]*self.ref_turning_speed                                   

        # cutoff control if robot rolls or pitches too much
        if abs(self.roll*180/math.pi) > 5 or abs(self.pitch*180/math.pi)>65:
            self.control_effort=[0.0, 0.0]

        
        print("Torques: " + str(self.control_effort[0]) + ", " + str(self.control_effort[1]))

        
        self.got_new_msg  = True

    def config_callback(self,config,level):
        self.Kp = config["Kp"]
        self.Ki = config["Ki"]
        self.Kd = config["Kd"]
        self.TLim = config["TLim"]
        self.On = config["On"]
        return config

if __name__ == '__main__':
    rospy.init_node('robot_controller')
    try:
        robot_controller()
    except rospy.ROSInterruptException: pass