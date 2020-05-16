#!/usr/bin/env python

# IMPORT FUNZIONALITA' ROS PER IL PYTHON
import rospy
# IMPORT LIBRERIA PER LA GENERAZIONE DELLE CURVE
import reeds_shepp
# IMPORT NECESSARI ALL'INVIO DEI MESSAGGI DI CONTROLLO DEL VEICOLO
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
# IMPORT FUNZIONE PER LA TRASFORMAZIONE DI COORDINATE
from tf.transformations import euler_from_quaternion
# IMPORT LIBRERIE PER CALCOLI 
import numpy as np
from scipy.integrate import odeint
from io_linearization import io_linearization_control_law



class Trajectory_control():
    
    # DEFINIZIONE DELLE VARIABILI GLOBALI DELLA CLASSE
    tol = 0.05
    msg1 = Twist()
    deltasx = Float64()
    deltadx = Float64()
    t = np.linspace(0, 100, 1000)
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

    slope_x = []
    slope_y = []
    SLOPE = []


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

        # DEFINIZIONE DELLA COORDINATA INIZIALE E DI QUELLA FINALE
        coordinate = [(0.0, 0.0, 0.0), (-2.0, 4.0, np.pi)]
        # PARAMETRI UTILI ALLA GENERAZIONE DELLE CURVE DI REEDSSHEPP
        step_size = 0.25
        rho = 5.8
        # GENERAZIONE DELLE CURVE
        qs = reeds_shepp.path_sample(coordinate[0], coordinate[1], rho, step_size)
        # ESTRAZIONE DEI PARAMETRI LUNGO I DUE ASSI E ARROTONDAMENTO ALLA TERZA DECIMALE
        xs = np.round([coordinate[0] for coordinate in qs],3)
        ys = np.round([coordinate[1] for coordinate in qs],3)

        v1 = []
        v2 = []
        v3 = []
        v4 = []
        v5 = []
        v6 = []
        v7 = []
        v8 = []
        v9 = []
        v10 = []
        v11 = []
        v12 = []

        segment_x = [v1, v2, v3, v4, v5, v6]
        segment_y = [v7, v8, v9, v10, v11, v12]
        i = 0
        j = 0
        initial_x = []
        initial_y = []
        final_x = []
        final_y = []

        if xs[i+1] > xs[i]:
            temp = True
        else:
            temp = False
        
        segment_x[j].append(xs[i])
        segment_y[j].append(ys[i])
        
        self.slope_x.append(temp)

        if ys[i+1] > ys[i]: 
            self.slope_y.append(True)
        else: 
            self.slope_y.append(False)

        while i < len(xs)-1:
            if temp:
                if xs[i+1] > xs[i]:
                    segment_x[j].append(xs[i+1])
                    segment_y[j].append(ys[i+1])
                else:
                    temp = False
                    self.slope_x.append(temp)

                    if ys[i+1] > ys[i]: 
                        self.slope_y.append(True)
                    else: 
                        self.slope_y.append(False)

                    j = j+1
                    final_x.append(xs[i])
                    final_y.append(ys[i])
            if temp == False: 
                if xs[i+1] < xs[i]:
                    segment_x[j].append(xs[i+1])
                    segment_y[j].append(ys[i+1])
                else:
                    temp = True
                    self.slope_x.append(temp)

                    if ys[i+1] > ys[i]: 
                        self.slope_y.append(True)
                    else: 
                        self.slope_y.append(False)

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

        print(segment_x)
        print(segment_y)
           
        while i <= j:
            
            if len(segment_x[i]) < 3 or len(segment_y[i]) < 3:                  
                segment_x[i].append(segment_x[i][0]+0.1)    
                segment_x[i].append(segment_x[i][0]-0.1)
                segment_y[i].append(segment_y[i][0]+0.1)    
                segment_y[i].append(segment_y[i][0]-0.1)
            


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

            i = i +1
                
            
        self.trajectory(R, x_c, y_c, N, initial_x, initial_y, final_x, final_y, segment_x, segment_y)
        #print(self.x_d)
        #print(self.y_d)



    def trajectory(self, R, x_c, y_c, N, initial_x, initial_y, final_x, final_y, segment_x, segment_y):
    
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
            j = 1
            k = []

            for element in x_d_temp:
                if self.slope_x[counter] and self.slope_y[counter]: #Controllo la direzione del tratto (se crescente o decrescente)
                    if x_d_temp[i] < initial_x[counter] or x_d_temp[i] > final_x[counter] or y_d_temp[i] < initial_y[counter] or y_d_temp[i] > final_y[counter] or x_d_temp[i] > x_d_temp[j] or y_d_temp[i] > y_d_temp[j]: 
                        k.append(i)

                elif self.slope_x[counter] == False and self.slope_y[counter]: 
                    if x_d_temp[i] > initial_x[counter] or x_d_temp[i] < final_x[counter] or y_d_temp[i] < initial_y[counter] or y_d_temp[i] > final_y[counter] or x_d_temp[i] < x_d_temp[j] or y_d_temp[i] > y_d_temp[j]: 
                        k.append(i)

                elif self.slope_x[counter] and self.slope_y[counter] == False: 
                    if x_d_temp[i] < initial_x[counter] or x_d_temp[i] > final_x[counter] or y_d_temp[i] > initial_y[counter] or y_d_temp[i] < final_y[counter] or x_d_temp[i] > x_d_temp[j] or y_d_temp[i] < y_d_temp[j]: 
                        k.append(i)
        
                else: 
                    if x_d_temp[i] > initial_x[counter] or x_d_temp[i] < final_x[counter] or y_d_temp[i] > initial_y[counter] or y_d_temp[i] < final_y[counter] or x_d_temp[i] < x_d_temp[j] or y_d_temp[i] < y_d_temp[j]: 
                        k.append(i)
                      
                i = i+1
                if j < len(x_d_temp)-1: 
                    j = j+1 
            
            # FUNZIONE DELETE PER RIMUOVERE GLI ELEMENTI DESIDERATI
            x_d_temp = np.delete(x_d_temp,k)
            y_d_temp = np.delete(y_d_temp,k)
            dotx_d_temp = np.delete(dotx_d_temp,k)
            doty_d_temp = np.delete(doty_d_temp,k)

            
            i = 0
            j = 1
            k = []
            for element in x_d_temp:
                if self.slope_x[counter]:
                    if x_d_temp[j] < x_d_temp[i]:
                        k.append(j)
                        i = i-1
                else:
                    if x_d_temp[j] > x_d_temp[i]:
                        k.append(j)
                        i = i-1
                i = i+1
                if j < len(x_d_temp)-1: 
                    j = j+1

            # FUNZIONE DELETE PER RIMUOVERE GLI ELEMENTI DESIDERATI
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
        rospy.sleep(0.1) #wait to fill q configuration
        max_t = self.t[len(self.t) - 1]
        len_t = len(self.t)
        for i in np.arange(0, len(self.x_d)):
            (y1, y2, theta) = self.get_point_coordinate(b)
            (self.v, self.w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b)
            # inverto direzione sterzata in caso di retromarcia
        
            if self.v < 0:
                self.w = -self.w
                
            print(self.SLOPE)

            err = self.get_error(i)
            print(err)
            #move robot
            self.sterzata()
            self.publish()

            rospy.sleep(max_t/len_t)

    def get_pose(self):
        #get robot position updated from callback
        x = self.q[0]
        #print(x)
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
        if abs(self.w) < 0.005:
            self.deltadx = 0
            self.deltasx = 0 
        else:
            R = (self.v/self.w)
            self.deltadx = np.arctan(self.b_meter/R)
            self.deltasx = self.deltadx
        
        

if __name__ == "__main__":
    try:
        
        tc=Trajectory_control()
        tc.trajectory_generation()
        tc.unicycle_linearized_control()

    except rospy.ROSInterruptException:
        pass

