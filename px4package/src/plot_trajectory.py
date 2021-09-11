#!/usr/bin/env python

import rospy
import mavros
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped


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


fig = plt.figure()
ax = Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(ax)
ax.set_xlim3d(-20, 0)
ax.set_ylim3d(-20, 0)
ax.set_zlim3d(0, 20)

 
def plot():
	rospy.init_node('offb_node', anonymous=True)
	#prev_state = current_state
	rate = rospy.Rate(20.0) # MUST be more then 2Hz
	
	rospy.loginfo("Hello")


	while not rospy.is_shutdown():
		
		ax.scatter3D(new_pose0.pose.position.x, new_pose0.pose.position.y, new_pose0.pose.position.z, color = "green", label = "interpolated points")
		ax.scatter3D(new_pose1.pose.position.x, new_pose1.pose.position.y, new_pose1.pose.position.z, color = "blue", label = "interpolated points")
		ax.scatter3D(new_pose2.pose.position.x, new_pose2.pose.position.y, new_pose2.pose.position.z, color = "cyan", label = "interpolated points")
		ax.scatter3D(new_pose3.pose.position.x, new_pose3.pose.position.y, new_pose3.pose.position.z, color = "red", label = "interpolated points")
		plt.pause(0.01)
		print(new_pose0.pose.position.x)

		rate.sleep()


    

rospy.loginfo("Hey")
#plt.show()


if __name__=='__main__':
    try:
        plot()
    except rospy.ROSInterruptException:
        pass
