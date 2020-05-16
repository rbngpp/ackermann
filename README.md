# ackermann
Car-like wehicle on Gazebo and path planner.

![ROS](https://img.shields.io/badge/ROS-MELODIC-9cf)
![PYTHON](https://img.shields.io/badge/PYTHON-inside-blueviolet)
![C++](https://img.shields.io/badge/C%2B%2B-inside-ff69b4)
![CMAKE](https://img.shields.io/badge/CMAKE-inside-lightgrey)


Step per clonare correttamente questo progetto:

1. Effettuare il GIT CLONE di questo repository nella cartella catkin_ws/src:

` cd catkin_ws/src `

` git clone https://github.com/rbngpp/ackermann.git `

2. Effettuare il catkin_make:

` cd ~/catkin_ws `

` catkin_make `

3. Installare la libreria readchar per il controllo tramite comandi W-A-S-D:

` pip install readchar ` 

4. Installare i ros-melodic-controllers:

` sudo apt-get install ros-melodic-ros-controllers `


Per lanciare la simulazione bisogna utilizzare la stringa seguente:

` roslaunch ackermann display.launch `

Per lanciare il controllo W-A-S-D: 

` rosrun ackermann wasd.py `

Per lanciare il controllo con curve di ReedsShepp:

` rosrun ackermann trajectory.py `

