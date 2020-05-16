 # CONTROLLO SE IL PRIMO TRATTO PREVEDA O MENO UN INCREMENTO LUNGO X, DUNQUE AGGIORNO SLOPE_X
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

while i < len(xs)-1: #effettuo il controllo su ciascun punto
   
    segment_x[j].append(xs[i+1]) # inserisco il nuovo valore all'interno dei punti appartenenti al tratto 
    segment_y[j].append(ys[i+1])
    
    i = i+1
    previous = case

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


"""
while i < len(xs)-1: #effettuo il controllo su ciascun punto
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