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


# callback method for state sub
current_state0 = State()
current_state1 = State()
current_state2 = State()
current_state3 = State()

new_pose0 = PoseStamped()
new_pose1 = PoseStamped()
new_pose2 = PoseStamped()
new_pose3 = PoseStamped()

#offb_set_mode = SetMode

def state_cb0(state):
    global current_state0
    current_state0 = state
def state_cb1(state):
    global current_state1
    current_state1 = state
def state_cb2(state):
    global current_state2
    current_state2 = state
def state_cb3(state):
    global current_state3
    current_state3 = state


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

local_pos_pub0 = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
local_pos_pub1 = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
local_pos_pub2 = rospy.Publisher('uav2/mavros/setpoint_position/local', PoseStamped, queue_size=1)
local_pos_pub3 = rospy.Publisher('uav3/mavros/setpoint_position/local', PoseStamped, queue_size=1)

rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, position_cb0)
rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, position_cb1)
rospy.Subscriber('uav2/mavros/local_position/pose', PoseStamped, position_cb2)
rospy.Subscriber('uav3/mavros/local_position/pose', PoseStamped, position_cb3)

rospy.Subscriber('uav0/mavros/state', State, state_cb0)
rospy.Subscriber('uav1/mavros/state', State, state_cb1)
rospy.Subscriber('uav2/mavros/state', State, state_cb2)
rospy.Subscriber('uav3/mavros/state', State, state_cb3)

#state_sub = rospy.Subscriber(mavros.get_topic('uav1','state'), State, state_cb)
arming_client0 = rospy.ServiceProxy('uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
arming_client1 = rospy.ServiceProxy('uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
arming_client2 = rospy.ServiceProxy('uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
arming_client3 = rospy.ServiceProxy('uav3/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

set_mode_client0 = rospy.ServiceProxy('uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
set_mode_client1 = rospy.ServiceProxy('uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
set_mode_client2 = rospy.ServiceProxy('uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
set_mode_client3 = rospy.ServiceProxy('uav3/mavros/set_mode', mavros_msgs.srv.SetMode)

pose0 = PoseStamped()
pose1 = PoseStamped()
pose2 = PoseStamped()
pose3 = PoseStamped()

pose0.pose.position.x = -19.5743655878585  #-10
pose0.pose.position.y = 5.20300810246330   #-5
pose0.pose.position.z = 3.05508686421701#10

pose1.pose.position.x = 4.32703055058928#0
pose1.pose.position.y = 6.44173554547888#-10
pose1.pose.position.z = -5.73692205795540#10

pose2.pose.position.x = -4.29358883931621#-5
pose2.pose.position.y = -0.848482919806163#0
pose2.pose.position.z = 1.99670842528129#-5

pose3.pose.position.x = 18.9149701471337#10
pose3.pose.position.y = -12.0894736462126#5
pose3.pose.position.z = 1.75594775649737#-5


threshold = 0.1

r1 = np.array([[-19.5743655878585, 5.20300810246330, 3.05508686421701]])
r2 = np.array([[4.32703055058928, 6.44173554547888, -5.73692205795540]])
r3 = np.array([[-4.29358883931621, -0.848482919806163, 1.99670842528129]])
r4 = np.array([[18.9149701471337, -12.0894736462126, 1.75594775649737]])
#print(r4)

r = np.concatenate((r1,r2,r3,r4), axis=None)
m = np.asmatrix(r).transpose()
# print(r)
# print(r.shape)
# print(type(r))

t_period = [0,50]
#t = np.linspace(0, 50, 1561) 

def myodefunc(t,r):
  L = np.array([[5, -5, 0, 0],[0, 4 ,-2.5 ,-1.5],[-1.5, 0, 3.8 ,-2.3],[-3.2, 0 ,-1.3, 4.5]])
  #L = np.array([[3, -3, 0, 0],[0, 2 ,-0.5 ,-1.5],[-1.5, 0, 1.8 ,-0.3],[-1.2, 0 ,-1.3, 2.5]])
  gp = 12.2307
  kp = 2
  R = np.array([[ -0.8116081,  0.0305878,  0.5834009],[ 0.5267739, -0.3934565,  0.7534595],[0.2525896,  0.9188343,  0.3032196 ]])
  #R = np.array([[ -0.5116081,  0.0305878,  0.2834009],[ 0.2267739, -0.1934565,  0.4534595],[0.2525896,  0.6188343,  0.0032196 ]])
  I = np.identity(12)
  K = np.kron(L,R)
  A = -gp*I - kp*K
  dr_dt = np.dot(A,r)
  return dr_dt


###########  Error plot    ################
plt.axis([0, 3, 0, 2])



def all_reached(x_cmd0, y_cmd0, z_cmd0, x_cmd1, y_cmd1, z_cmd1, x_cmd2, y_cmd2, z_cmd2, x_cmd3, y_cmd3, z_cmd3):
    if abs(new_pose0.pose.position.x - x_cmd0) <= threshold and abs(new_pose0.pose.position.y - y_cmd0) <= threshold and abs(new_pose0.pose.position.z - z_cmd0) <= threshold:
        if abs(new_pose1.pose.position.x - x_cmd1) <= threshold and abs(new_pose1.pose.position.y - y_cmd1) <= threshold and abs(new_pose1.pose.position.z - z_cmd1) <= threshold:
            if abs(new_pose2.pose.position.x - x_cmd2) <= threshold and abs(new_pose2.pose.position.y - y_cmd2) <= threshold and abs(new_pose2.pose.position.z - z_cmd2) <= threshold:
                if abs(new_pose3.pose.position.x - x_cmd3) <= threshold and abs(new_pose3.pose.position.y - y_cmd3) <= threshold and abs(new_pose3.pose.position.z - z_cmd3) <= threshold:
                    return True


def get_distance(point1, point2):
    return np.linalg.norm(point1 - point2)


def position_control():
    rospy.init_node('offb_node', anonymous=True)
    #prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub0.publish(pose0)
        local_pos_pub1.publish(pose1)
        local_pos_pub2.publish(pose2)
        local_pos_pub3.publish(pose3)
        rate.sleep()
   

    # wait for FCU connection
    while not current_state0.connected and not current_state1.connected and not current_state2.connected and not current_state3.connected:
        rate.sleep()

    rospy.loginfo("Hello")
    go = 0
    j = 0
    flag = 0
    armed = 0
    offb = 0

    check45 = integrate.RK45(myodefunc, 0, r, 100, max_step = 0.01)   #max_step = 0.005 for smooth curve
    # x = check45.y
    # time = check45.t
    #check45.step() #step has no input
    x_E45 = check45.y

    x_cmd0 = x_E45[0:12][0] +22   #12
    y_cmd0 = x_E45[0:12][1] +22
    z_cmd0 = x_E45[0:12][2] + 20

    x_cmd1 = x_E45[0:12][3] +22
    y_cmd1 = x_E45[0:12][4] +22
    z_cmd1 = x_E45[0:12][5] + 20

    x_cmd2 = x_E45[0:12][6] +22
    y_cmd2 = x_E45[0:12][7] +22
    z_cmd2 = x_E45[0:12][8] + 20

    x_cmd3 = x_E45[0:12][9] +22
    y_cmd3 = x_E45[0:12][10] +22
    z_cmd3 = x_E45[0:12][11] + 20


    last_request = rospy.get_rostime()
    t0 = rospy.get_rostime()
    
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state0.mode != "OFFBOARD" and current_state1.mode != "OFFBOARD" and current_state2.mode != "OFFBOARD" and current_state3.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)) and armed == 0:
            set_mode_client0(base_mode=0, custom_mode="OFFBOARD")
            set_mode_client1(base_mode=0, custom_mode="OFFBOARD")
            set_mode_client2(base_mode=0, custom_mode="OFFBOARD")
            set_mode_client3(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
            rospy.loginfo("OFFBOARD SET")
            offb = 1
        else:
            if not current_state0.armed and not current_state1.armed and not current_state2.armed and not current_state3.armed and (now - last_request > rospy.Duration(5.)) and armed == 0:
               arming_client0(True)
               arming_client1(True)
               arming_client2(True)
               arming_client3(True)
               last_request = now 
               rospy.loginfo("ARMED")
               armed = 1


        point2 = np.array((x_cmd0, y_cmd0, z_cmd0))

        x_E45 = check45.y

        x_cmd0 = x_E45[0:12][0] -12    #-5
        y_cmd0 = x_E45[0:12][1] -12
        z_cmd0 = x_E45[0:12][2] + 20

        x_cmd1 = x_E45[0:12][3] -12
        y_cmd1 = x_E45[0:12][4] -12
        z_cmd1 = x_E45[0:12][5] + 20

        x_cmd2 = x_E45[0:12][6] -12
        y_cmd2 = x_E45[0:12][7] -12
        z_cmd2 = x_E45[0:12][8] + 20

        x_cmd3 = x_E45[0:12][9] -12
        y_cmd3 = x_E45[0:12][10]-12
        z_cmd3 = x_E45[0:12][11] + 20


        pose0.pose.position.x = x_cmd0
        pose0.pose.position.y = y_cmd0
        pose0.pose.position.z = z_cmd0

        pose1.pose.position.x = x_cmd1
        pose1.pose.position.y = y_cmd1
        pose1.pose.position.z = z_cmd1

        pose2.pose.position.x = x_cmd2
        pose2.pose.position.y = y_cmd2
        pose2.pose.position.z = z_cmd2

        pose3.pose.position.x = x_cmd3
        pose3.pose.position.y = y_cmd3
        pose3.pose.position.z = z_cmd3


        point1 = np.array((new_pose0.pose.position.x, new_pose0.pose.position.y, new_pose0.pose.position.z))
        dist = get_distance(point1, point2)

        local_pos_pub0.publish(pose0)
        local_pos_pub1.publish(pose1)
        local_pos_pub2.publish(pose2)
        local_pos_pub3.publish(pose3)

        if all_reached(x_cmd0, y_cmd0, z_cmd0, x_cmd1, y_cmd1, z_cmd1, x_cmd2, y_cmd2, z_cmd2, x_cmd3, y_cmd3, z_cmd3) and go == 0:
            go = 1
            #rospy.sleep(6)
            t0 = rospy.get_rostime()


        # if all_reached(x_cmd0, y_cmd0, z_cmd0, x_cmd1, y_cmd1, z_cmd1, x_cmd2, y_cmd2, z_cmd2, x_cmd3, y_cmd3, z_cmd3):
        #     check45.step()
        #     rospy.loginfo(j)
        #     rospy.loginfo(check45.t)
        #     rospy.loginfo(check45.step_size)
        #     j = j + 1

        if go == 1 and (rospy.get_rostime() - t0 >= rospy.Duration(0.3)):  ##0.5 for smoother curve
            #plt.scatter(check45.t, dist)
            #plt.pause(0.01)
            check45.step()
            t0 = rospy.get_rostime()
            rospy.loginfo(j)
            rospy.loginfo(x_E45)
            rospy.loginfo("Time = " + str(check45.t))
            j = j + 1



        rate.sleep()
        

if __name__=='__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass