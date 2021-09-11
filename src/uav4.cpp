#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <px4package/InitStatus.h>
#define uavid "uav3"
#define uavid3 uav3


int id = 4;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped feedback;
px4package::InitStatus updater;
px4package::InitStatus checker;


bool i_reached = false;
bool all_reached = false;

float threshold = 0.1;
//float a=0.4, b=0.8, w = 0.5;  //max w for circle = 0.7 otherwise the drone cuts the circle and reduces the radius with tilting to a large extent
float x_init=0;    
float y_init=4;
float z_init=2;
double to;


void reached_cb(const px4package::InitStatus::ConstPtr& msg)
{
  //gdsgfsf
  checker = *msg;
  if(checker.uav0==true&&checker.uav1==true&&checker.uav2==true&&checker.uav3==true&&all_reached==false)
  {
    all_reached = true;
    to=ros::Time::now().toSec();
  }
  else if (all_reached == false)
  {
    all_reached = false;
  }
}
void whether_reached()
{
  
  if (i_reached==true)
  {
    updater.uav0 = checker.uav0;
    updater.uav1 = checker.uav1;
    updater.uav2 = checker.uav2;
    updater.uav3 = checker.uav3;
    updater.uavid3 = true;
    // reached_pub.publish(updater);
  }
  else
  {
    updater.uav0 = checker.uav0;
    updater.uav1 = checker.uav1;
    updater.uav2 = checker.uav2;
    updater.uav3 = checker.uav3;
    updater.uavid3 = false;
  }
}
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    feedback = *msg;
    //ROS_INFO("in position_cb");
    if(fabs(feedback.pose.position.x-x_init)<= threshold && fabs(feedback.pose.position.y-y_init)<=threshold && i_reached==false)
    {
      i_reached=true;
      //whether_reached();
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
    ros::Subscriber reached_sub = nh.subscribe<px4package::InitStatus>
            ("/reached", 10, reached_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    //whether_reached();
    
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 10;
    pose.pose.position.y = 5;
    pose.pose.position.z = -5;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandTOL land;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    
    while(ros::ok())
    {
        // whether_reached();
        // reached_pub.publish(updater);
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


    local_pos_pub.publish(pose); 

      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}