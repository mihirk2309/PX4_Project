#!/usr/bin/env python

import rospy
import mavros
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
import random
from itertools import count
import pandas as pd


new_pose0 = PoseStamped()
new_pose1 = PoseStamped()
new_pose2 = PoseStamped()
new_pose3 = PoseStamped()



def position_cb0(Pose):
    global new_pose0
    new_pose0 = Pose
def position_cb1(Pose):
    global new_pose1
    new_pose1 = Pose
def position_cb2(Pose):
    global new_pose2
    new_pose2 = Pose
def position_cb3(Pose):
    global new_pose3
    new_pose3 = Pose


rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, position_cb0)
rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, position_cb1)
rospy.Subscriber('uav2/mavros/local_position/pose', PoseStamped, position_cb2)
rospy.Subscriber('uav3/mavros/local_position/pose', PoseStamped, position_cb3)


r1 = np.array([[-10, -5, 10]])
r2 = np.array([[0, -10, 10]])
r3 = np.array([[-5, 0, -5]])
r4 = np.array([[10, 5, -5]])

r = np.concatenate((r1,r2,r3,r4), axis=None)
m = np.asmatrix(r).transpose()


t_period = [0,50]


def myodefunc(t,r):
  L = np.array([[5, -5, 0, 0],[0, 4 ,-2.5 ,-1.5],[-1.5, 0, 3.8 ,-2.3],[-3.2, 0 ,-1.3, 4.5]])
  gp = 12.2307
  kp = 2
  R = np.array([[ -0.8116081,  0.0305878,  0.5834009],[ 0.5267739, -0.3934565,  0.7534595],[0.2525896,  0.9188343,  0.3032196 ]])
  I = np.identity(12)
  K = np.kron(L,R)
  A = -gp*I - kp*K
  dr_dt = np.dot(A,r)
  return dr_dt

check45 = integrate.RK45(myodefunc, 0, r, 10, max_step = 0.005)


plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()


def animate(i):
    x_vals.append(next(index))
    y = 

    plt.cla()

    plt.plot(x_vals, y_vals, label='Error magnitude')

    plt.legend(loc='upper left')
    plt.tight_layout()


ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()

 
def plot():
    rospy.init_node('offb_node', anonymous=True)
    #prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    
    rospy.loginfo("Hello")


    while not rospy.is_shutdown():
        
        x_E45=check45.y

        x_cmd0 = x_E45[0:12][0]
        y_cmd0 = x_E45[0:12][1]
        z_cmd0 = x_E45[0:12][2]

        

        rate.sleep()


    

rospy.loginfo("Hey")
#plt.show()


if __name__=='__main__':
    try:
        plot()
    except rospy.ROSInterruptException:
        pass
