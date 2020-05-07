#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
import numpy as np
#from unicycle import unicycle_error_model, control
from scipy.integrate import odeint




class Trajectory_control():
    #attributes
    t = []
    x_d = 2
    y_d = 0
    theta_d = 0
    v_d = []
    w_d = []
    q=[0, 0, 0]
    q_i=[]
    q_f=[]
    err = []

    

    #methods
    def __init__(self):
        rospy.init_node('trajectory', anonymous=True) #make node
        rospy.loginfo("Starting node Trajectory control")
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        rospy.sleep(1)
        #self.twist_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.twist_pub = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10) 
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)


    #current robot pose
    def odometryCb(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.get_angle_pose(msg.pose.pose)
        self.q = np.array([x, y, theta])
        return self.q

    #compute angle from quaternion
    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        return theta

    def get_error(self, T):
        #slide 80 LDC
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        #rospy.loginfo("x={} y={} th={}".format(x,y,theta))
        #compute error
        e1 = (self.x_d - x) * np.cos(theta) + (self.y_d - y) * np.sin(theta)
        e2 = -(self.x_d - x) * np.sin(theta) + (self.y_d - y) * np.cos(theta)
        e3 = self.theta_d - theta
        self.err = np.array([e1, e2, e3])

        return self.err

    def  stampa(self):
        rospy.loginfo('Errore: %s', self.err)
        rospy.loginfo('Posizione Corrente: %s', self.q)
    
    #postprocessing 
    def unicicle_publish_control_var(self):
        msg = Twist()
        msg.linear.x = self.err[0]
        self.twist_pub.publish(msg)
            
"""
            #move robot
            self.send_velocities(v, w, theta_t)
            rospy.sleep(10./1000)
        
        #stop after time
        self.send_velocities(0,0,0)


    #publish v, w
    def send_velocities(self, v, w, theta):
        twist_msg = Twist() # Creating a new message to send to the robot
        twist_msg.linear.x = v * np.cos(theta)
        twist_msg.linear.y = v * np.sin(theta)
        twist_msg.angular.z = w
        self.twist_pub.publish(twist_msg)
"""


if __name__ == "__main__":
    try:
        tc=Trajectory_control()
        tc.get_error(0)
        #tc.unicicle_publish_control_var()
        tc.stampa()
        
    except rospy.ROSInterruptException:
        pass
