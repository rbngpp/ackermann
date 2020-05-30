#!/usr/bin/env python
# ------------------------------------------------
# IMPORT
# ------------------------------------------------

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
import matplotlib.pyplot as plt


# ------------------------------------------------
# DEFINIZIONE DELLA CLASSE PRINCIPALE
# ------------------------------------------------

class Trajectory_control():
    
    # ------------------------------------------------
    # DEFINIZIONE DELLE VARIABILI GLOBALI DELLA CLASSE
    # ------------------------------------------------

    # Definzione dei messaggi per il controllo del veicolo
    msg1 = Twist()
    deltasx = Float64()
    deltadx = Float64()
    # Definzione degli istanti di tempo su cui calcolare la traiettoria
    t = []
    # Definzione dei parametri della traiettoria desiderata
    x_d = []
    y_d = []
    theta_d = []
    v_d = []
    w_d = []
    dotx_d = []
    doty_d = []
    # Traiettoria, posizione iniziale, posizione finale, errore
    q=[]
    q_i=[]
    q_f=[]
    err = []
    # Parametri di progetto: interasse, passo, passo scalato, veocita' lineare ed angolare
    a = 0.21
    b = 0.25
    b_meter = b/10
    v = 0
    w = 0
    # Rate di pubblicazione sul topic
    rate = 0
    # Vettori booleani con informazioni sulla pendenza dei tratti parabolici lungo gli assi X ed Y
    slope_x = []
    slope_y = []
    SLOPE = []

    # ------------------------------------------------
    # METODI
    # ------------------------------------------------

    def __init__(self):
        rospy.init_node('trajectory', anonymous=True) # creazione del nodo
        self.rate = rospy.Rate(10) # definizione del rate di pubblicazione
        rospy.loginfo("Starting node Trajectory control") # messaggio di segnalazione avvio procedura
        self.twist_pub = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10) # definizione del canale di pubblicazione delle ruote posteriori
        self.left_pub = rospy.Publisher('/sinistra/command', Float64, queue_size=10) # definzione del canale di pubblicazione della ruota anteriore sinistra
        self.right_pub = rospy.Publisher('/destra/command', Float64, queue_size=10) # definzione del canale di pubblicazione della ruota anteriore destra
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb) # definizione del canale di sottoscrizione per la posizione attuale del veicolo
    
    # Metodo per ottenere la posizione attuale del veicolo
    def odometryCb(self,msg):
        x = msg.pose.pose.position.x # posizione lungo x rispetto all'origine
        y = msg.pose.pose.position.y # posizione lungo y rispetto all'origine
        theta = self.get_angle_pose(msg.pose.pose)  # angolo theta tra la direzione del veicolo e l'asse x
        self.q = np.array([x, y, theta]) # inserimento di x,y,theta all'interno del vettore "q"
        return self.q

    # Calcolo dell'angolo theta in coordinate RPY (dunque calcolo della coordinata Y) a partire da un quaternione
    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x, # estrapolazione dei dati letti dal canale
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q) # calcolo RPY tramite funzione apposita

        theta = yaw # definizione di theta (Y)
        return theta

    # Generazione delle circonferenze di cui si compone la traiettoria desiderata a partire da punti iniziale e finale e dalle curve di ReedsShepp
    def trajectory_generation(self):

        # DEFINIZIONE DELLA COORDINATA INIZIALE E DI QUELLA FINALE
        coordinate = [(0.0,0.0,0.0),(3.0,-4.0,np.pi)]
        # PARAMETRI UTILI ALLA GENERAZIONE DELLE CURVE DI REEDSSHEPP
        step_size = 0.01823
        rho = 5.8
        # GENERAZIONE DELLE CURVE DI REEDS SHEPP
        qs = reeds_shepp.path_sample(coordinate[0], coordinate[1], rho, step_size)
        tmax = np.round(reeds_shepp.path_length(coordinate[0], coordinate[1], rho),2)
        # ESTRAZIONE DEI PARAMETRI LUNGO I DUE ASSI E ARROTONDAMENTO ALLA TERZA DECIMALE
        xs = np.round([coordinate[0] for coordinate in qs],3)
        ys = np.round([coordinate[1] for coordinate in qs],3)  
        theta = np.round([coordinate[2] for coordinate in qs],3)
        self.t = np.linspace(0,tmax,len(xs))
        # Richiamo il metodo "trajectory" per l'estrazione dei segmenti rappresentanti le curve di Reeds Shepp a partire dalle circonferenze calcolate
        
        v_d_val = 0.5 # m/s
        element = 0
        u1 = np.zeros(len(xs))

        while element < len(theta)-1: 
            if theta[element] < np.pi/2 and theta[element] > -np.pi/2: 
                if xs[element] < xs[element+1]: 
                    u1[element] = 1
                else: 
                    u1[element] = -1
            else: 
                if xs[element] > xs[element+1]: 
                    u1[element] = 1
                else: 
                    u1[element] = -1
            element = element+1

        self.dotx_d = u1*np.cos(theta) 
        self.doty_d =  u1*np.sin(theta)
        self.v_d = np.sqrt(self.dotx_d**2 + self.doty_d**2)
        self.theta_d = np.arctan2(self.doty_d, self.dotx_d)
        self.x_d = xs
        self.y_d = ys


            
    # Metodo per ottenere le coordinate y1,y2,theta del veicolo a partire da x,y,theta
    def get_point_coordinate(self, b):
        # Ottengo la posizione del veicolo tramite callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        # Calcolo delle coordinate da considerare
        y1 = x + b * np.cos(theta)
        y2 = y + b * np.sin(theta)
        return [y1, y2, theta]

    # Metodo per il controllo del veicolo
    def unicycle_linearized_control(self):
        b = 0.04 # Distanza tra il punto B ed il punto di contatto P
        rospy.sleep(0.1) # Attendo la callback
        max_t = self.t[len(self.t) - 1] # Calcolo il numero totale di istanti di tempo su cui effettuare il controllo
        len_t = len(self.t)
        for i in np.arange(0, len(self.x_d)):
            (y1, y2, theta) = self.get_point_coordinate(b) # Ottengo le coordinate per ogni punto
            (self.v, self.w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b) # Effettuo il controllo
            
            # Inverto direzione sterzata in caso di retromarcia
            if self.v < 0:
                self.w = -self.w

            # Ottengo l'errore tra la posizione attuale e quella desiderata  
            err = self.get_error(i)
            
            # Calcolo gli angoli di sterzata per le ruote anteriori
            self.sterzata()

            # Pubblico i valori calcolati nei canali opportuni
            self.publish()

            # Attendo prima di procedere con il calcolo del punto successivo
            rospy.sleep(max_t/len_t)

    # Metodo per ottenere la posa del veicolo
    def get_pose(self):
        # Ottengo la posa x,y,theta da callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        return np.array([x, y, theta])

    # Metodo per il calcolo dell'errore
    def get_error(self, T):
        (x, y, theta) = self.get_pose()
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y) * np.sin(theta)
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y) * np.cos(theta)
        e3 = 0
        err = np.array([e1, e2, e3])
        print(err)
        return err

    
    # Metodo per pubblicare i messaggi sui canali opportuni per il movimento e la sterzata del veicolo
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
 
        
   
    # Metodo per aggiornare gli angoli di sterzata delle due ruote anteriori
    def sterzata(self):
        if abs(self.w) < 0.005:
            self.deltadx = 0
            self.deltasx = 0 
        else:
            R = (self.v/self.w)
            self.deltadx = np.arctan(self.b_meter/R)
            self.deltasx = self.deltadx

# Funzione contenente la legge di controllo lineare
def io_linearization_control_law(y1, y2, theta, y1d, y2d, doty1d, doty2d, b):
        # Define the two control gains. Notice we can define "how fast" we track on y_1 and y_2 _independently_

        k_1 = 2.7
        k_2 = 2.7
       
        #return virtual input doty1, doty2
        u_1 = doty1d + k_1*(y1d - y1)
        u_2 = doty2d + k_2*(y2d - y2)

        #return control input v, w
        v = np.cos(theta) * u_1 + np.sin(theta) * u_2
        w = u_2/b * np.cos(theta) - u_1/b *np.sin(theta) #dottheta

        return np.array([v, w])       
        

if __name__ == "__main__":
    try:
        tc=Trajectory_control() # Istanzio la classe
        tc.trajectory_generation() # Genero la traiettoria
        tc.unicycle_linearized_control() # Applico il controllo

    except rospy.ROSInterruptException:
        pass

