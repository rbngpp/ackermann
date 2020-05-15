#!/usr/bin/env python

import rospy
import reeds_shepp
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
    v_d = []
    w_d = []
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

        coordinate = [(0.0, 0.0, 0.0), (3.0, 4.0, np.pi/2)]
        step_size = 0.25
        rho = 5.8
        qs = reeds_shepp.path_sample(coordinate[0], coordinate[1], rho, step_size)
        xs = np.round([coordinate[0] for coordinate in qs],3)
        ys = np.round([coordinate[1] for coordinate in qs],3)

        v1 = []
        v2 = []
        v3 = []
        v4 = []
        v5 = []
        v6 = []

        segment_x = [v1, v2, v3]
        segment_y = [v4, v5, v6]
        i = 0
        j = 0
        initial_x = []
        initial_y = []
        final_x = []
        final_y = []
        slope = []

        if xs[i+1] > xs[i]:
            temp = True
            segment_x[j].append(xs[i])
            segment_y[j].append(ys[i])
        else:
            temp = False
        
        slope.append(temp)

        while i < len(xs)-1:
            if temp:
                if xs[i+1] > xs[i]:
                    segment_x[j].append(xs[i+1])
                    segment_y[j].append(ys[i+1])
                else:
                    temp = False
                    slope.append(temp)
                    j = j+1
                    final_x.append(xs[i])
                    final_y.append(ys[i])
            if temp == False: 
                if xs[i+1] < xs[i]:
                  segment_x[j].append(xs[i+1])
                  segment_y[j].append(ys[i+1])
                else:
                    temp = True
                    slope.append(temp)
                    j = j+1
                    final_x.append(xs[i])
                    final_y.append(ys[i])
            i = i+1 

        final_x.append(xs[i])
        final_y.append(ys[i])

        i = 0
        N = j+1 # Numero tratti
        R = np.zeros(N)
        x_c = np.zeros(N)
        y_c = np.zeros(N)  
           
        while i <= j:
            Ax = segment_x[i][0]
            Bx = segment_x[i][1]
            Cx = segment_x[i][2]
            Ay = segment_y[i][0]
            By = segment_y[i][1]
            Cy = segment_y[i][2]

            A = [Ax, Ay]
            B = [Bx, By]
            C = [Cx, Cy]
            
            var = np.array([[A[0],A[1],1], [B[0],B[1],1], [C[0],C[1],1]])

            notA = -(A[0]**2)-(A[1]**2)
            notB = -(B[0]**2)-(B[1]**2)
            notC = -(C[0]**2)-(C[1]**2)
            noti = np.array([notA, notB, notC])
            soluzione = np.linalg.solve(var, noti)
            R[i] = np.sqrt((soluzione[0]**2/4)+(soluzione[1]**2/4)-soluzione[2])


            x_c[i] = -soluzione[0]/2
            y_c[i] = -soluzione[1]/2
            
            initial_x.append(Ax)
            initial_y.append(Ay)

            i = i+1 
        
        
        self.trajectory(R, x_c, y_c, N, slope, initial_x, initial_y, final_x, final_y, segment_x, segment_y)



    def trajectory(self, R, x_c, y_c, N, slope, initial_x, initial_y, final_x, final_y, segment_x, segment_y):
       
        """        
        x_d = [0.0, 0.249, 0.499, 0.747, 0.995, 1.240, 1.483, 1.723, 1.960, 2.193, 2.384]
        y_d = [0.0, 0.005, 0.021, 0.048, 0.085, 0.134, 0.192, 0.262, 0.341, 0.430, 0.512]

        x_d1 = [2.16   1.94   1.725  1.516  1.314  1.118  1.08]
        y_d1 = [0.403  0.285  0.156  0.019 -0.127 -0.282 -0.311]

        x_d2 = [1.262  1.437
                     1.603  1.762  1.912  2.053  2.186  2.309  2.422  2.526  2.62   2.704
                            2.778  2.841  2.894  2.936  2.968  2.989  2.999]
        y_d2 = [-0.14   0.039
                    0.225  0.418  0.618  0.824  1.036  1.254  1.477  1.704  1.936  2.171
                            2.41   2.652  2.896  3.143  3.391  3.64   3.889]
        """
        v_d_val = 0.5 # m/s
        counter = 0 
        self.x_d = np.asarray(self.x_d)
        self.y_d = np.asarray(self.y_d)
        self.dotx_d = np.asarray(self.dotx_d)
        self.doty_d = np.asarray(self.doty_d)
        self.v_d = np.asarray(self.v_d)
        self.w_d = np.asarray(self.w_d)
        while counter < N:
            w_d_val = v_d_val/R[counter]
            x_d_temp = R[counter] * np.cos(w_d_val * self.t) + x_c[counter]
            y_d_temp = R[counter] * np.sin(w_d_val * self.t) + y_c[counter]
            dotx_d_temp = -R[counter]*w_d_val*np.sin(w_d_val* self.t)
            doty_d_temp =  R[counter]*w_d_val*np.cos(w_d_val* self.t)

            i = 0
            k = []

            for element in x_d_temp:
                if slope[counter]: #Controllo la direzione del tratto (se crescente o decrescente)
                    if x_d_temp[i] < initial_x[counter] or y_d_temp[i] < initial_y[counter] or y_d_temp[i] > final_y[counter]: #modificare estremi (<0 or <0 or >2.15)
                        k.append(i)
                else:
                    if x_d_temp[i] > initial_x[counter] or y_d_temp[i] > initial_y[counter] or y_d_temp[i] < final_y[counter]: #modificare estremi (<0 or <0 or >2.15)
                        k.append(i)

                i = i+1
            
            x_d_temp = np.delete(x_d_temp,k)
            y_d_temp = np.delete(y_d_temp,k)
            dotx_d_temp = np.delete(dotx_d_temp,k)
            doty_d_temp = np.delete(doty_d_temp,k)

            v_d_temp = np.sqrt(dotx_d_temp**2 + doty_d_temp**2)
            theta_d_temp = np.arctan2(doty_d_temp, dotx_d_temp)
            w_d_temp = w_d_val * np.ones(len(self.t))

            
            self.x_d = np.append(self.x_d, x_d_temp)
            self.y_d = np.append(self.y_d, y_d_temp)
            self.dotx_d = np.append(self.dotx_d, dotx_d_temp)
            self.doty_d = np.append(self.doty_d, doty_d_temp)
            self.v_d = np.append(self.v_d, v_d_temp)
            self.theta_d = np.append(self.theta_d, theta_d_temp)
            self.w_d = np.append(self.w_d, w_d_temp)
            counter = counter +1 


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
            #print(err)
            #move robot
            self.sterzata()
            self.publish()

            rospy.sleep(max_t/len_t)

    def get_pose(self):
        #get robot position updated from callback
        x = self.q[0]
        print(x)
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

