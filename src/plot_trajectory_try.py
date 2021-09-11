#!/usr/bin/env python

import rospy
import mavros
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
import time


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

rospy.init_node('offb_node', anonymous=True)


fig = plt.figure()
ax = Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(ax)
ax.set_xlim3d(-20, 0)
ax.set_ylim3d(-20, 0)
ax.set_zlim3d(0, 20)

def plot(self):
    ax.scatter(new_pose0.pose.position.x, new_pose0.pose.position.y, new_pose0.pose.position.z, color = "green", label = "interpolated points")
    ax.scatter(new_pose1.pose.position.x, new_pose1.pose.position.y, new_pose1.pose.position.z, color = "blue", label = "interpolated points")
    ax.scatter(new_pose2.pose.position.x, new_pose2.pose.position.y, new_pose2.pose.position.z, color = "cyan", label = "interpolated points")
    ax.scatter(new_pose3.pose.position.x, new_pose3.pose.position.y, new_pose3.pose.position.z, color = "red", label = "interpolated points")
    print(new_pose0.pose.position.x)
    #ax.figure.canvas.draw()

    

ani = FuncAnimation(plt.gcf(), plot, interval=100)
plt.show()


# while 1:
#     print(new_pose0.pose.position.x)


#     while not rospy.is_shutdown():

#         if count%5 == 0:
#             ax.scatter3D(new_pose0.pose.position.x, new_pose0.pose.position.y, new_pose0.pose.position.z, color = "green", label = "interpolated points")
#             ax.scatter3D(new_pose1.pose.position.x, new_pose1.pose.position.y, new_pose1.pose.position.z, color = "blue", label = "interpolated points")
#             ax.scatter3D(new_pose2.pose.position.x, new_pose2.pose.position.y, new_pose2.pose.position.z, color = "cyan", label = "interpolated points")
#             ax.scatter3D(new_pose3.pose.position.x, new_pose3.pose.position.y, new_pose3.pose.position.z, color = "red", label = "interpolated points")
#         #if count%10 == 0:
#             plt.pause(0.0001)
#         #plt.show()
#         print(new_pose0.pose.position.x)
#         count = count + 1
#         time.sleep(0.1)

#         rate.sleep()

    
    

# #rospy.loginfo("Hey")
# #plt.show()


# if __name__=='__main__':
#     try:
#         plot()
#     except rospy.ROSInterruptException:
#         plt.show()
#         pass
