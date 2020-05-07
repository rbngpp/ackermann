#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Twist

def get_angle_pose(quaternion_pose):
    q1 = [quaternion_pose.orientation.x,
        quaternion_pose.orientation.y,
        quaternion_pose.orientation.z,
        quaternion_pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q1)

    theta = yaw
    return theta

def callback(msg):
    global q
    qf = [2,0,0]
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = get_angle_pose(msg.pose.pose)
    q = np.array([x,y,theta]) 
    
    rospy.loginfo(rospy.get_caller_id() + 'Posizione lungo X: %s', x)
    #rospy.loginfo(rospy.get_caller_id() + 'Posizione lungo Y: %s', y)
    #rospy.loginfo(rospy.get_caller_id() + 'Orientamento Theta: %s', theta)
    err = get_error(0,qf,q)
    #rospy.loginfo(rospy.get_caller_id() + 'Errore: %s', err)


    return q

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ground_truth/state', Odometry, callback)
    rospy.spin()
    return q

def get_error(T,qf,q):
    #slide 80 LDC
    #get robot position updated from callback
    x = q[0]
    y = q[1]
    theta = q[2]
    x_d = qf[0]
    y_d = qf[1]
    theta_d = qf[2]
    #rospy.loginfo("x={} y={} th={}".format(x,y,theta))
    #compute error
    e1 = (x_d - x) * np.cos(theta) + (y_d - y) * np.sin(theta)
    e2 = -(x_d - x) * np.sin(theta) + (y_d - y) * np.cos(theta)
    e3 = theta_d - theta
    err = np.array([e1, e2, e3])
    return err

if __name__ == '__main__':
    pub = rospy.Publisher('/posteriori/cmd_vel', Twist, queue_size=10)
    listener()
    rospy.spin()
    #qf = [2, 0, 0]
    #err = get_error(0,qf,q)
    #rospy.loginfo(rospy.get_caller_id() + 'Errore: %s', err)