#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
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


        self.E = 0.0        # proportional error
        self.Eold = 0.0     # previous error
        self.Eint = 0.0     # integral error
        self.Edot = 0.0     # derivative error
        self.Kp = 1.0     # proportional gain
        self.Ki = 1.0     # integral gain
        self.Kd = 1.0      # derivative gain

        self.control_effort = [0.0,0.0]
        self.TLim = 15.0

        self.euler= [0.0,0.0,0.0]
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0


        # create publishers and subscribers
        sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.pid_controller)
        sub_fallen = rospy.Subscriber("teeterbot/fallen_over", Bool, self.reset_controller)
        pub_right_wheel = rospy.Publisher("teeterbot/right_torque_cmd", Float64, queue_size=10)
        pub_left_wheel  = rospy.Publisher("teeterbot/left_torque_cmd", Float64, queue_size=10)

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
            self.Eold = 0
            self.E = 0
            self.control_effort = [0.0,0.0]
            self.upright = False
            self.got_new_msg  = True
        else:
            self.upright = True
            self.got_new_msg  = True

    def pid_controller(self,msg):

        # convert orientation to roll pitch yaw
        quat = (msg.pose[1].orientation.x,
                msg.pose[1].orientation.y,
                msg.pose[1].orientation.z,
                msg.pose[1].orientation.w)

        self.euler = tf.transformations.euler_from_quaternion(quat)
        self.roll = self.euler[0]
        self.pitch = self.euler[1]
        self.yaw = self.euler[2]
    
        # calculate error
        self.E = self.pitch

        # calculate integral error
        self.Eint = self.Eint + self.E

        # calculate derivative error
        self.Edot = self.E  - self.Eold

        # update previous error
        self.Eold = self.E

        # calculate control effort
        control = self.Kp*self.E + self.Ki*self.Eint + self.Kd*self.Edot
        self.control_effort[0] = math.copysign(1,control)*min(abs(control),self.TLim)
        self.control_effort[1] = math.copysign(1,control)*min(abs(control),self.TLim)


        self.got_new_msg  = True

    def config_callback(self,config,level):
        self.Kp = config["Kp"]
        self.Ki = config["Ki"]
        self.Kd = config["Kd"]
        #self.TLim = config["TLim"]
        self.On = config["On"]
        return config

if __name__ == '__main__':
    rospy.init_node('robot_controller')
    try:
        robot_controller()
    except rospy.ROSInterruptException: pass