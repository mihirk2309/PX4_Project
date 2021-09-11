#!/usr/bin/env python

import rospy
import mavros
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import integrate
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

new_pose0 = PoseStamped()
new_pose1 = PoseStamped()
new_pose2 = PoseStamped()
new_pose3 = PoseStamped()

#offb_set_mode = SetMode


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





# Creating figure
#fig = plt.figure(figsize = (10, 7))
#ax = plt.axes(projection ="3d")
#ax = plt.axis([0, 20, 0, 20, 0, 20])
fig = plt.figure()
ax = Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(ax)
ax.set_xlim3d(0, 20)
ax.set_ylim3d(0, 20)
ax.set_zlim3d(0, 20)

 
def position_control():
	rospy.init_node('offb_node', anonymous=True)
	#prev_state = current_state
	rate = rospy.Rate(20.0) # MUST be more then 2Hz
	
	rospy.loginfo("Hello")


	while not rospy.is_shutdown():
		
		ax.scatter3D(new_pose0.pose.position.x, new_pose0.pose.position.y, new_pose0.pose.position.z, color = "green", label = "interpolated points")
		ax.scatter3D(new_pose1.pose.position.x, new_pose1.pose.position.y, new_pose1.pose.position.z, color = "blue", label = "interpolated points")
		ax.scatter3D(new_pose2.pose.position.x, new_pose2.pose.position.y, new_pose2.pose.position.z, color = "red", label = "interpolated points")
		ax.scatter3D(new_pose3.pose.position.x, new_pose3.pose.position.y, new_pose3.pose.position.z, color = "cyan", label = "interpolated points")
		plt.pause(0.01)
		print(new_pose0.pose.position.x)

		rate.sleep()


#for i in range(100):
#while 1:
    
    

rospy.loginfo("Hey")
#plt.show()




if __name__=='__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
