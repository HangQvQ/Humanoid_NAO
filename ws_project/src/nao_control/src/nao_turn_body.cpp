// This is a service to control NAO to turn the body for specific degrees

#include "ros/ros.h"
#include "nao_control/TurnAround.h"
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher walk_pub;
ros::Publisher head_pub;

void head(float theta)
{
    /* 
        rostopic pub -1 /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed '{header: auto, joint_names: ["HeadYaw"], joint_angles: [0], speed: 0.1}
    */
    naoqi_bridge_msgs::JointAnglesWithSpeed msg;
    msg.joint_names.push_back("HeadYaw");
    msg.joint_angles.push_back(theta);
    msg.speed= 0.1;		

    head_pub.publish(msg);
}

void walker(double x, double y, double theta)
{
    ROS_INFO("[TURN] start Walking");
    geometry_msgs::Pose2D msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    walk_pub.publish(msg);
}

bool serviceCallback(nao_control::TurnAround::Request  &req, nao_control::TurnAround::Response &res)
{
    ros::Duration(1).sleep();
    walker(0,0,req.mode);
    ros::Duration(1).sleep();
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_around_service");
    ros::NodeHandle n;

    // Publisher for the walking
    walk_pub=n.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

    // Publisher for the head rotating
    head_pub=n.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1);

    // Service for turning around
    ros::ServiceServer service = n.advertiseService("turnaround", serviceCallback);
    
    
    ros::spin();

    return 0;
}