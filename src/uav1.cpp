/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * 
 * 
 * //**************CIRCLE****************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <px4package/InitStatus.h>
#define uavid "uav0"
#define uavid1 uav0

mavros_msgs::State current_state;
geometry_msgs::PoseStamped feedback;
px4package::InitStatus updater;
px4package::InitStatus checker;

bool reached = false;
float threshold = 0.1;
float a=2, b=2, w = 0.5;  //max w for circle = 0.7 otherwise the drone cuts the circle and reduces the radius with tilting to a large extent
float x_init=0;    
float y_init=4;
float z_init=2;
    
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    feedback = *msg;
    if(fabs(feedback.pose.position.x-x_init)<= threshold && fabs(feedback.pose.position.y-y_init)<=threshold && reached==false)
    {
      reached=true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, uavid);
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (uavid"/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (uavid"/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (uavid"/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (uavid"/mavros/set_mode");
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            (uavid"/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (uavid"/mavros/local_position/pose", 10, position_cb);
    ros::ServiceClient  land_client = nh.serviceClient<mavros_msgs::CommandTOL> 
      (uavid"/mavros/cmd/land");
    ros::Publisher reached_pub = nh.advertise<px4package::InitStatus>
           ("/reached", 10);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 4;
    pose.pose.position.z = 2;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    int armed = 0;
    float V_max=0.5;
    float x_cmd, y_cmd,z_cmd;
    double t;
    x_cmd = x_init;
    y_cmd = y_init;
    z_cmd = z_init;


    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        if(reached == true){

            t=ros::Time::now().toSec();
            //ros::Time::setNow(t);
             
            ROS_INFO("time = %f\n", t);
            ROS_INFO("x_cmd = %f\n",x_cmd);
            ROS_INFO("y_cmd = %f\n",y_cmd);
            //ros::Duration(0.5).sleep();

            // if(fabs(feedback.pose.position.x-x_cmd) <= 1 && fabs(feedback.pose.position.y-y_cmd) <= 1)
            // {
            //     x_cmd=4*sin(a*t);     //Calculate Commanding X
            //     y_cmd=4*cos(b*t);     //Calculate Commanding Y
            //     z_cmd = 2; //Publishing Pose
            // }

            x_cmd=4*sin(w*t);     //Calculate Commanding X
            y_cmd=4*cos(w*t);     //Calculate Commanding Y
            z_cmd = 2;

            pose.pose.position.x = x_cmd;     //Setpoint Position
            pose.pose.position.y = y_cmd;       
            pose.pose.position.z = z_cmd;
            pose.pose.orientation.w = 0; //Setpoint Orientation
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.x = 0;
            local_pos_pub.publish(pose); 

        }
        else{
            pose.pose.position.x = x_init;     //Setpoint Position
            pose.pose.position.y = y_init;       
            pose.pose.position.z = z_init;
            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }
//x=A*cos(alpha*wt) y=B*sin(Beta*wt)
//Alpha/beta = 1:2 //figure 8


    return 0;
}