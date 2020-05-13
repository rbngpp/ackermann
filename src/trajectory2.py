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
from trajectory_generation import Trajectory_generation



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
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)
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

    def trajectory_generation(self, trajectory):
        tg = Trajectory_generation()
        if(trajectory == "cubic"):
            #Cubic_trajectory
            q_i = np.array([0.,0., 3.14/2]) #Initial posture (x_i,y_i,theta_i)
            q_f = np.array([3.,6., 0.])    #Final posture   (x_f,y_f,theta_f)
            init_final_velocity = 2
            (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d) = tg.cubic_trajectory(q_i, q_f, init_final_velocity, self.t)   
        elif(trajectory == "eight"):
            #Eight trajectory
            (self.x_d, self.y_d, self.dotx_d, self.doty_d) = tg.eight_trajectory(self.t)
        elif (trajectory == "cyrcular"):    
            #Cyrcular_trajectory
            (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d, self.dotx_d, self.doty_d) = tg.cyrcular_trajectory(self.t)


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
        for i in np.arange(0, len(self.t)):
            (y1, y2, theta) = self.get_point_coordinate(b)
            (self.v, self.w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b)
            #print("linear:{} and angular:{}".format(self.v, self.w))
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
        #rospy.loginfo("x={} y={} th={}".format(x,y,theta))
        #compute error
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y) * np.sin(theta)
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y) * np.cos(theta)
        e3 = 0#self.theta_d[T] - theta
        err = np.array([e1, e2, e3])
        return err


    def  stampa(self):
        self.get_error()
        print (self.err)
        #rospy.loginfo('Errore: %s', self.err)
        #rospy.loginfo('Posizione Corrente: %s', self.q)
    
    #postprocessing 
    def publish(self):
         
        self.msg1.linear.x = self.v
        self.msg1.linear.y = 0.0
        self.msg1.linear.z = 0.0
        self.msg1.angular.x = 0.0
        self.msg1.angular.y = 0.0
        self.msg1.angular.z = 0.0
        
        # rospy.loginfo('INVIO DATI')
        # rospy.loginfo(self.msg1)
        # rospy.loginfo(self.deltasx)
        # rospy.loginfo(self.deltadx)

        # rospy.loginfo('POSIZIONE CORRENTE')
        # rospy.loginfo(self.q[0])
        # rospy.loginfo(self.q[1])
        
        self.left_pub.publish(self.deltasx) 
        self.right_pub.publish(self.deltadx)
        self.twist_pub.publish(self.msg1)
        #self.rate.sleep()
        #self.get_error()
        
   
    
    def sterzata(self):
        if abs(self.w) < 0.01:
            self.deltadx = 0
            self.deltasx = 0 
        else:
            R = self.v/self.w
            self.deltadx = np.arctan(self.b/R)
            self.deltasx = self.deltadx
        
        

if __name__ == "__main__":
    try:
        """
        tc=Trajectory_control()
        tc.get_error()
        while (abs(tc.q[0]-tc.x_d) > tc.tol or abs(tc.q[1]-tc.y_d) > tc.tol):
            u_t = un.control(tc.err, tc.v_d, tc.w_d)
            tc.v = tc.v_d*np.cos(tc.err[2])-u_t[0]
            tc.w = tc.w_d-u_t[1]
            tc.w = -tc.w
            tc.sterzata()
            tc.publish()
        
        tc.stampa()
        """
        tc=Trajectory_control()
        tc.t = np.linspace(0, 100, 1000)
        trajectory = "eight"  #cubic, eight, cyrcular
        tc.trajectory_generation(trajectory)
        #tc.unicicle_nonLinear_control()
        tc.unicycle_linearized_control()

    except rospy.ROSInterruptException:
        pass

