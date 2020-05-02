# ackermann
Car-like wehicle on Gazebo and path planner.

*A seguito di modifiche alla CMakeList si consiglia di effettuare nuovamente il catkin_make [29.04.2020]*


Pacchetti da installare una volta:

` pip install readchar ` 

` sudo apt-get install ros-melodic-ros-controllers `

Per lanciare la simulazione bisogna utilizzare la stringa seguente:

` roslaunch ackermann display.launch `

Per lanciare il controllo W-A-S-D: 

` rosrun ackermann wasd_movement.py `
