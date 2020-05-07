#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
import numpy as np
#from unicycle import unicycle_error_model, control
from scipy.integrate import odeint




class Trajectory_control():
    #attributes
    tol = 0.01
    msg1 = Twist()
    deltasx = Float64()
    deltadx = Float64()
    t = []
    x_d = 1
    y_d = 1
    theta_d = 0
    v_d = []
    w_d = []
    q=[0, 0, 0]
    q_i=[]
    q_f=[]
    err = []
    a = 0.21
    b = 0.25


    #methods
    def __init__(self):
        rospy.init_node('trajectory', anonymous=True) #make node
        rospy.loginfo("Starting node Trajectory control")
        self.twist_pub = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10) 
        self.left_pub = rospy.Publisher('sinistra/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('destra/command', Float64, queue_size=10)
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)
        self.sub()

    def sub(self):
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        rospy.sleep(0.1) 


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

    def get_error(self):
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
    def publish(self):
        self.msg1.linear.x = self.err[0]
        self.msg1.linear.y = 0.0
        self.msg1.linear.z = 0.0
        self.msg1.angular.x = 0.0
        self.msg1.angular.y = 0.0
        self.msg1.angular.z = 0.0

        ipo = np.sqrt((self.x_d-self.q[0])**2 + (self.y_d-self.q[1])**2)
        angolo = np.arctan2((self.y_d-self.q[1]),(self.x_d-self.q[0]))
        rospy.loginfo(angolo)

        if self.y_d-self.q[1] > 0: 
            self.deltasx = angolo
            R = self.b/np.tan(self.deltasx) + self.a/2
            self.deltadx = np.arctan(self.b/(R+self.a/2))
        else: 
            self.deltadx = angolo
            R = self.b/np.tan(self.deltadx) - self.a/2
            self.deltasx = np.arctan(self.b/(R-self.a/2))
    
        rospy.loginfo(self.deltasx)
        rospy.loginfo(self.deltadx)
        
        self.twist_pub.publish(self.msg1)
        self.left_pub.publish(self.deltasx)
        self.right_pub.publish(self.deltadx)

    def control(self):
        self.get_error()
        while abs(self.err[0]) > self.tol:
            self.publish()
            self.sub()
            self.get_error()
            self.stampa()
                     
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
        tc.control()
        
        
    except rospy.ROSInterruptException:
        pass
