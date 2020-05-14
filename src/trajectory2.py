#!/usr/bin/env python

import rospy
import unicycle as un
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
import numpy as np
#from unicycle import unicycle_error_model, control
from scipy.integrate import odeint
from io_linearization import io_linearization_control_law



class Trajectory_control():
    #attributes
    tol = 0.05
    msg1 = Twist()
    deltasx = Float64()
    deltadx = Float64()
    t = []
    x_d = []
    y_d = []
    theta_d = []
    v_d = 2
    w_d = 2
    dotx_d = []
    doty_d = []
    q=[]
    q_i=[]
    q_f=[]
    err = []
    a = 0.21
    b = 0.25
    b_meter = b/10
    v = 0
    w = 0
    rate = 0


    #methods
    def __init__(self):
        rospy.init_node('trajectory', anonymous=True) #make node
        self.rate = rospy.Rate(10)    
        rospy.loginfo("Starting node Trajectory control")
        self.twist_pub = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10) 
        self.left_pub = rospy.Publisher('/sinistra/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/destra/command', Float64, queue_size=10)
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
    
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


    def trajectory_generation(self):
        
        (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d, self.dotx_d, self.doty_d) = self.trajectory()



    def trajectory(self):
        
        A = [0.0, 0.0]
        B = [0.499, 0.021]
        C = [0.995, 0.085]  
        
        var = np.array([[A[0],A[1],1], [B[0],B[1],1], [C[0],C[1],1]])

        notA = -(A[0]**2)-(A[1]**2)
        notB = -(B[0]**2)-(B[1]**2)
        notC = -(C[0]**2)-(C[1]**2)
        noti = np.array([notA, notB, notC])
        soluzione = np.linalg.solve(var, noti)

        R = np.sqrt((soluzione[0]**2/4)+(soluzione[1]**2/4)-soluzione[2])

        v_d_val = 0.5 # m/s
        w_d_val = v_d_val/R

        x_c = -soluzione[0]/2
        y_c = -soluzione[1]/2

        """        
        x_d = [0.0, 0.249, 0.499, 0.747, 0.995, 1.240, 1.483, 1.723, 1.960, 2.193, 2.384, 2.159]
        y_d = [0.0, 0.005, 0.021, 0.048, 0.085, 0.134, 0.192, 0.262, 0.341, 0.430, 0.512, 0.403]
        """

        x_d = R * np.cos(w_d_val * self.t) + x_c
        y_d = R * np.sin(w_d_val * self.t) + y_c
        dotx_d = -R*w_d_val*np.sin(w_d_val* self.t)
        doty_d =  R*w_d_val*np.cos(w_d_val* self.t)

        i = 0
        k = []
        for element in x_d:
            if x_d[i] < 0 or y_d[i] < 0 or y_d[i] > 2.15: 
                k.append(i)
            i = i+1
        
        x_d = np.delete(x_d,k)
        y_d = np.delete(y_d,k)
        dotx_d = np.delete(dotx_d,k)
        doty_d = np.delete(doty_d,k)

        print(np.shape(x_d))
        print(y_d)

        v_d = np.sqrt(dotx_d**2 + doty_d**2)
        theta_d = np.arctan2(doty_d, dotx_d)
        w_d = w_d_val * np.ones(len(self.t))

        return [x_d, y_d, v_d, w_d, theta_d, dotx_d, doty_d]



    def get_point_coordinate(self, b):
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        #robot point cooordinate to consider
        y1 = x + b * np.cos(theta)
        y2 = y + b * np.sin(theta)
        return [y1, y2, theta]

    def unicycle_linearized_control(self):
        # Distance of point B from the point of contact P
        b = 0.02
        rospy.sleep(0.1) #wait to fill q cinfiguration
        max_t = self.t[len(self.t) - 1]
        len_t = len(self.t)
        for i in np.arange(0, len(self.x_d)):
            (y1, y2, theta) = self.get_point_coordinate(b)
            (self.v, self.w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b)
            err = self.get_error(i)
            print(err)
            #move robot
            self.sterzata()
            self.publish()

            rospy.sleep(max_t/len_t)

    def get_pose(self):
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        return np.array([x, y, theta])

    def get_error(self, T):
        #slide 80 LDC
        (x, y, theta) = self.get_pose()
      
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y) * np.sin(theta)
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y) * np.cos(theta)
        e3 = 0#self.theta_d[T] - theta
        err = np.array([e1, e2, e3])
        return err


    def  stampa(self):
        self.get_error()
        print (self.err)

    
    #postprocessing 
    def publish(self):
         
        self.msg1.linear.x = self.v
        self.msg1.linear.y = 0.0
        self.msg1.linear.z = 0.0
        self.msg1.angular.x = 0.0
        self.msg1.angular.y = 0.0
        self.msg1.angular.z = 0.0
        
        self.left_pub.publish(self.deltasx) 
        self.right_pub.publish(self.deltadx)
        self.twist_pub.publish(self.msg1)
 
        
   
    
    def sterzata(self):
        if abs(self.w) < 0.05:
            self.deltadx = 0
            self.deltasx = 0 
        else:
            R = (self.v/self.w)
            self.deltadx = np.arctan(self.b_meter/R)
            self.deltasx = self.deltadx
        
        

if __name__ == "__main__":
    try:
        
        tc=Trajectory_control()
        tc.t = np.linspace(0, 100, 1000)
        tc.trajectory_generation()
        tc.unicycle_linearized_control()

    except rospy.ROSInterruptException:
        pass

