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
    t = np.linspace(0, 100, 1000)
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
        coordinate = [(0.0, 0.0, 0.0), (-4.0, 3.0, np.pi)]
        # PARAMETRI UTILI ALLA GENERAZIONE DELLE CURVE DI REEDSSHEPP
        step_size = 0.25
        rho = 5.8
        # GENERAZIONE DELLE CURVE DI REEDS SHEPP
        qs = reeds_shepp.path_sample(coordinate[0], coordinate[1], rho, step_size)
        # ESTRAZIONE DEI PARAMETRI LUNGO I DUE ASSI E ARROTONDAMENTO ALLA TERZA DECIMALE
        xs = np.round([coordinate[0] for coordinate in qs],3)
        ys = np.round([coordinate[1] for coordinate in qs],3)

        # INIZIALIZZAZIONE DEI VETTORI "segment_x" E "segment_y" PER DISTINGUERE LE VARIE CIRCONFERENZE
        v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12 = [],[],[],[],[],[],[],[],[],[],[],[]
        segment_x = [v1, v2, v3, v4, v5, v6]
        segment_y = [v7, v8, v9, v10, v11, v12]

        # INIZIALIZZAZIONE DEI VETTORI CONTENENTI GLI ELEMENTI INIZIALI E FINALI DI CIASCUN TRATTO
        i = 0
        j = 0
        initial_x = []
        initial_y = []
        final_x = []
        final_y = []

        # CONTROLLO SE IL PRIMO TRATTO PREVEDA O MENO UN INCREMENTO LUNGO X, DUNQUE AGGIORNO SLOPE_X
        if xs[i+1] > xs[i]:
            temp = True
        else:
            temp = False
        
        segment_x[j].append(xs[i])
        segment_y[j].append(ys[i])
        
        self.slope_x.append(temp)

        # CONTROLLO SE IL PRIMO TRATTO PREVEDA O MENO UN INCREMENTO LUNGO Y, DUNQUE AGGIORNO SLOPE_Y
        if ys[i+1] > ys[i]: 
            self.slope_y.append(True)
        else: 
            self.slope_y.append(False)

        # CONTROLLO I TRATTI SUCCESSIVI AL PRIMO
        print(xs)
        print(ys)
        
        case = 0
        if xs[i+1] > xs[i] and ys[i+1] > ys[i]:
            case = 1
        elif xs[i+1] < xs[i] and ys[i+1] > ys[i]:
            case = 2
        elif xs[i+1] < xs[i] and ys[i+1] < ys[i]:
            case = 3
        else:
            case = 4

        segment_x[j].append(xs[i])
        segment_y[j].append(ys[i])

        if case == 1:
            self.slope_x.append(True)
            self.slope_y.append(True)
        elif case == 2:
            self.slope_x.append(False)
            self.slope_y.append(True)
        elif case == 3:
            self.slope_x.append(False)
            self.slope_y.append(False)
        else:
            self.slope_x.append(True)
            self.slope_y.append(False)
        
        previous = case

        while i < len(xs)-1: #effettuo il controllo su ciascun punto
        

            if xs[i+1] > xs[i] and ys[i+1] > ys[i]:
                case = 1
            elif xs[i+1] < xs[i] and ys[i+1] > ys[i]:
                case = 2
            elif xs[i+1] < xs[i] and ys[i+1] < ys[i]:
                case = 3
            else:
                case = 4
            
            if previous != case:
                if case == 1:
                    self.slope_x.append(True)
                    self.slope_y.append(True)           
                elif case == 2:
                    self.slope_x.append(False)
                    self.slope_y.append(True)
                elif case == 3:
                    self.slope_x.append(False)
                    self.slope_y.append(False)
                else:
                    self.slope_x.append(True)
                    self.slope_y.append(False)
            
                j = j+1 # incremento il numero dei tratti totali 
                final_x.append(xs[i]) # aggiorno il valore finale del tratto rispetto all'asse X
                final_y.append(ys[i]) # aggiorno il valore finale del tratto rispetto all'asse Y
            else: 
                segment_x[j].append(xs[i+1]) # inserisco il nuovo valore all'interno dei punti appartenenti al tratto 
                segment_y[j].append(ys[i+1])
            
            i = i+1
            previous = case

            


            """
            if temp: #qualora il punto precedente appartenga ad un tratto che si incrementa in senso positivo
                if xs[i+1] > xs[i]: # qualora il tratto si stia ancora incrementando
                    segment_x[j].append(xs[i+1]) # inserisco il nuovo valore all'interno dei punti appartenenti al tratto 
                    segment_y[j].append(ys[i+1])
                else: # in caso contrario considero il nuovo tratto con pendenza opposta rispetto al precedente
                    temp = False
                    self.slope_x.append(temp)
                    if ys[i+1] > ys[i]: # valuto il verso del tratto anche rispetto all'asse Y
                        self.slope_y.append(True)
                    else: 
                        self.slope_y.append(False)
                    j = j+1 # incremento il numero dei tratti totali 
                    final_x.append(xs[i]) # aggiorno il valore finale del tratto rispetto all'asse X
                    final_y.append(ys[i]) # aggiorno il valore finale del tratto rispetto all'asse Y
            if temp == False: #qualora il punto precedente appartenga ad un tratto che si incrementa in senso negativo
                if xs[i+1] < xs[i]: #qualora il decremento non sia ancora terminato
                    segment_x[j].append(xs[i+1]) # inserisco il nuovo valore all'interno dei punti appartenenti al tratto 
                    segment_y[j].append(ys[i+1])
                else: # in caso contrario considero il nuovo tratto con pendenza opposta rispetto al precedente
                    temp = True
                    self.slope_x.append(temp)

                    if ys[i+1] > ys[i]:   # valuto il verso del tratto anche rispetto all'asse Y
                        self.slope_y.append(True)
                    else: 
                        self.slope_y.append(False)
                    j = j+1 # incremento il numero dei tratti totali 
                    final_x.append(xs[i]) # aggiorno il valore finale del tratto rispetto all'asse X
                    final_y.append(ys[i]) # aggiorno il valore finale del tratto rispetto all'asse Y        
            i = i+1 
            """
            

        final_x.append(xs[i]) # aggiorno il valore finale del tratto rispetto all'asse X
        final_y.append(ys[i]) # aggiorno il valore finale del tratto rispetto all'asse Y

        print(segment_x)
        print(segment_y)

        print(final_x)
        print(final_y)

        i = 0 # ripristino un contatore generico al valore nullo
        N = j+1 # Numero totale dei tratti
        R = np.zeros(N) # inizializzo il vettore contente i raggi di ciascuna traiettoria
        x_c = np.zeros(N) # inizializzo il vettore contente x_c per ciacsuna traiettoria
        y_c = np.zeros(N) # inizializzo il vettore contente y_c per ciascuna traiettoria
           
        # Definisco il sistema per il calcolo delle circonferenze a partire dai primi tre punti di ciascuna traiettoria   
        while i <= j:
            # Calcolo A,B,C contenenti le tre equazioni
            Ax = segment_x[i][0]
            Bx = segment_x[i][1]
            Cx = segment_x[i][2]
            Ay = segment_y[i][0]
            By = segment_y[i][1]
            Cy = segment_y[i][2]
            A = [Ax, Ay]
            B = [Bx, By]
            C = [Cx, Cy]
            # Inserisco tutto in un'unica matrice "var"    
            var = np.array([[A[0],A[1],1], [B[0],B[1],1], [C[0],C[1],1]])
            # Calcolo il vettore dei termini noti
            notA = -(A[0]**2)-(A[1]**2)
            notB = -(B[0]**2)-(B[1]**2)
            notC = -(C[0]**2)-(C[1]**2)
            noti = np.array([notA, notB, notC])
            # Calcoloo la soluzione con solver lineare
            soluzione = np.linalg.solve(var, noti)
            # Estraggo il raggio per ciascuna delle circonferenze
            R[i] = np.sqrt((soluzione[0]**2/4)+(soluzione[1]**2/4)-soluzione[2])

            # Estraggo x_c ed y_c per ciascuna delle circonferenze
            x_c[i] = -soluzione[0]/2
            y_c[i] = -soluzione[1]/2
            # Aggiorno initial_x ed initial_y con i primi valori di ciascun tratto per i due assi
            initial_x.append(Ax)
            initial_y.append(Ay)
            # incrmeento il counter e passo alla circonferenza a partire dal tratto successivo
            i = i +1
                
        # Richiamo il metodo "trajectory" per l'estrazione dei segmenti rappresentanti le curve di Reeds Shepp a partire dalle circonferenze calcolate
        self.trajectory(R, x_c, y_c, N, initial_x, initial_y, final_x, final_y, segment_x, segment_y)


    # Metodo per il calcolo della traiettoria (curve di Reeds Shepp) da far seguire al veicolo
    def trajectory(self, R, x_c, y_c, N, initial_x, initial_y, final_x, final_y, segment_x, segment_y):
        
        # Inizializzazione dei parametri utili al calcolo delle curve
        v_d_val = 0.5 # m/s
        counter = 0 
        self.x_d = np.asarray(self.x_d)
        self.y_d = np.asarray(self.y_d)
        self.dotx_d = np.asarray(self.dotx_d)
        self.doty_d = np.asarray(self.doty_d)
        self.v_d = np.asarray(self.v_d)
        self.w_d = np.asarray(self.w_d)
        print(initial_x)
        print(initial_y)
        print(final_x)
        print(final_y)


        # Calcolo di w_d,x_d,y_d,dotx_d e doty_d per ciascuna delle circonferenze individuate
        while counter < N:
            w_d_val = v_d_val/R[counter]
            x_d_temp = R[counter] * np.cos(w_d_val * self.t) + x_c[counter]
            y_d_temp = R[counter] * np.sin(w_d_val * self.t) + y_c[counter]
            dotx_d_temp = -R[counter]*w_d_val*np.sin(w_d_val* self.t)
            doty_d_temp =  R[counter]*w_d_val*np.cos(w_d_val* self.t)

            
            # Inizializzazione di contatori ed elementi utili
            i = 0
            j = 1
            k = []

            # Rimozione da ciascuna circonferenza dei punti non facenti parte delle curve di Reeds Shepp
            # Il controllo si effettua a seconda dell'andamento della curva rispetto agli assi X ed Y
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
            
            # Rimozione degli elementi superflui associati agli altri vettori
            x_d_temp = np.delete(x_d_temp,k)
            y_d_temp = np.delete(y_d_temp,k)
            dotx_d_temp = np.delete(dotx_d_temp,k)
            doty_d_temp = np.delete(doty_d_temp,k)

            
            # Secondo controllo per rimozione di eventuali punti in eccesso
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

            # Rimozione degli elementi superflui associati agli altri vettori
            x_d_temp = np.delete(x_d_temp,k)
            y_d_temp = np.delete(y_d_temp,k)
            dotx_d_temp = np.delete(dotx_d_temp,k)
            doty_d_temp = np.delete(doty_d_temp,k)
            
            v_d_temp = np.sqrt(dotx_d_temp**2 + doty_d_temp**2)
            theta_d_temp = np.arctan2(doty_d_temp, dotx_d_temp)
            w_d_temp = w_d_val * np.ones(len(self.t))
            
            # Inserimento dei dati temporanei nei vettori corrispondenti prima di effettuare il calcolo del segmento successivo
            self.x_d = np.append(self.x_d, x_d_temp)
            self.y_d = np.append(self.y_d, y_d_temp)
            self.dotx_d = np.append(self.dotx_d, dotx_d_temp)
            self.doty_d = np.append(self.doty_d, doty_d_temp)
            self.v_d = np.append(self.v_d, v_d_temp)
            self.theta_d = np.append(self.theta_d, theta_d_temp)
            self.w_d = np.append(self.w_d, w_d_temp)

           
            # Incremento del contatore per passare al calcolo del segmento successivo
            counter = counter +1 
            
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
        b = 0.02 # Distanza tra il punto B ed il punto di contatto P
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
        k_1 = 0.5
        k_2 = 0.5
       
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

