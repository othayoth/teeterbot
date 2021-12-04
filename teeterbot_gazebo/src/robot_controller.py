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


class robot_controller():
    def __init__(self):
        # robot's balancing state        
        self.upright = True

        # robot state vector = [forward velocity, body tilt, body tilt velocity, turning velocity]
        self.state = [0.0, 0.0, 0.0, 0.0]
        
        # reference trajectory
        self.ref_forward_speed = 0.0  # reference forward speed specified by the user        
        self.ref_turning_speed = 0.0  # reference turning speed specified by the user
        # output trajectory achieved by the robot
        self.out_forward_speed = 0.0  # output forward speed achieved by the robot
        self.out_turning_speed = 0.0  # output turning speed achieved by the robot
        self.output_velocity = Twist() # twist to store the messages
        
        
        # right and left wheel speeds (to calculate turning speed)
        self.rw_speed = 0.0
        self.lw_speed = 0.0
        
        # State feedback gain1s from LQR with integral action (generated from matlab code)
        self.K_r = [ -23.3150, -100.5295,  -33.9993,    7.3668,   15.8114,  -15.8114]
        self.K_l = [ -23.3150, -100.5295,  -33.9993,   -7.3668,   15.8114,   15.8114]
        self.K_r =   [-4.3099,  -25.7769,   -5.9101,    2.3672,    2.2361,   -2.2361]
        self.K_l =   [-4.3099,  -25.7769,   -5.9101,   -2.3672,    2.2361,    2.2361]

        # control torques for right and left wheels
        self.control_effort = [0.0,0.0]
        
        # body orientations
        self.euler = [0.0,0.0,0.0]
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        
        # create subscribers  
        self.sub_model       = rospy.Subscriber("gazebo/link_states", ModelStates, self.state_feedback_controller)
        self.sub_reference   = rospy.Subscriber("/cmd_vel", Twist, self.reference_callback)
        self.sub_fallen      = rospy.Subscriber("teeterbot/fallen_over", Bool, self.reset_controller)        
        self.sub_right_speed = rospy.Subscriber("teeterbot/right_wheel_speed", Float64, self.right_wheel_speed_callback)        
        self.sub_left_speed  = rospy.Subscriber("teeterbot/left_wheel_speed", Float64, self.left_wheel_speed_callback)                
        # create publishers
        self.pub_right_wheel = rospy.Publisher( "teeterbot/right_torque_cmd", Float64,queue_size=10)
        self.pub_left_wheel  = rospy.Publisher( "teeterbot/left_torque_cmd", Float64,queue_size=10)
        self.pub_output_vel  = rospy.Publisher( "teeterbot/output_vel",Twist,queue_size=10)
        self.pub_command_vel = rospy.Publisher( "teeterbot/commanded_vel",Twist,queue_size=10)
        
                
        # Main while loop.
        while not rospy.is_shutdown():                        
            # engage wheels to balance robot if it is not fallen over
            if self.upright:
                self.pub_right_wheel.publish(self.control_effort[0])
                self.pub_left_wheel.publish(self.control_effort[1])                        
            # disengage wheels to balance robot when fallen over   
            else:
                self.pub_right_wheel.publish(0.0)
                self.pub_left_wheel.publish(0.0)
                                
    # callback -- robot's fallen state
    def reset_controller(self,msg):            
        if(msg.data):
            print("teeterbot fell over; stop control effort")            
            self.control_effort = [0.0,0.0]
            self.pub_right_wheel.publish(0.0)
            self.pub_left_wheel.publish(0.0)                                   
            self.upright = False
        else:
            print("teeterbot upright; begin control effort")            
            self.upright = True
            
    
    # callback -- reference trajectory -- update references as the forward/turning speed commands 
    def reference_callback(self,msg):                
        self.ref_forward_speed = msg.linear.x
        self.ref_turning_speed = msg.angular.z       
        print("reference trajectory updated")

    # callbacks -- wheel speeds -- read current wheel speeds
    def right_wheel_speed_callback(self,msg):                
        self.rw_speed = msg.data
    def left_wheel_speed_callback(self,msg):                
        self.lw_speed = msg.data
        

    # main controller -- callback -- teeterbot state
    def state_feedback_controller(self,msg):
        
        # convert teterbot orientation to roll pitch yaw
        quat = (msg.pose[1].orientation.x,
                msg.pose[1].orientation.y,
                msg.pose[1].orientation.z,
                msg.pose[1].orientation.w)
        self.euler = tf.transformations.euler_from_quaternion(quat)                        
               
        #----- observed states--------------------------------------------

        # forward velocity (following is conversion to body fixed frame)
        self.state[0] = (msg.twist[1].linear.x*math.cos(self.euler[2]) +        
                         msg.twist[1].linear.y*math.sin(self.euler[2]))
        #self.state[0] = 0.2*(self.rw_speed + self.lw_speed)/2     
        # note -- either of the above would work, but the first one is preferred as it is independent of wheel slip assumption
        
        # body pitch
        self.state[1] = self.euler[1]                  
        
        # body pitch velocity (following is conversion to body fixed frame)
        self.state[2] = (-msg.twist[1].angular.x*math.sin(self.euler[2]) +        
                          msg.twist[1].angular.y*math.cos(self.euler[2]))
        
        # turning velocity calculated from wheels
        self.state[3] = 0.2*(self.rw_speed - self.lw_speed)/(2*0.5)     

        #-----------------------------------------------------------------  

        # publish output        
        self.output_velocity.linear.x  = self.state[0]
        self.output_velocity.angular.z = self.state[3]
        self.pub_output_vel.publish(self.output_velocity)

        
        # control effort based on state feedback control with integral action
        self.control_effort[0] = -(self.K_r[0]*self.state[0] + 
                                   self.K_r[1]*self.state[1] +
                                   self.K_r[2]*self.state[2] +
                                   self.K_r[3]*self.state[3] +        
                                   self.K_r[4]*(self.ref_forward_speed-self.state[0]) + # augmented state variable : error in forward velocity                                
                                   self.K_r[5]*(self.ref_turning_speed-self.state[3]))  # augmented state variable : error in turning velocity                                   
        self.control_effort[1] = -(self.K_l[0]*self.state[0] + 
                                   self.K_l[1]*self.state[1] +
                                   self.K_l[2]*self.state[2] +
                                   self.K_l[3]*self.state[3] +        
                                   self.K_l[4]*(self.ref_forward_speed-self.state[0]) + # augmented state variable : error in forward velocity                                
                                   self.K_l[5]*(self.ref_turning_speed-self.state[3]))  # augmented state variable : error in turning velocity                                                                      
        

        

        # cutoff control if robot rolls or pitches too much
        if abs(self.roll*180/math.pi) > 5 or abs(self.pitch*180/math.pi)>45:
            self.control_effort=[0.0, 0.0]

        
        
        
if __name__ == '__main__':
    rospy.init_node('robot_controller')
    try:
        robot_controller()
        
    except rospy.ROSInterruptException: pass