// This is a service to control NAO to turn the Head to look around

#include "ros/ros.h"
#include "nao_control/LookAround.h"
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <geometry_msgs/Pose2D.h>

#define HEADYAW_LEFT_LIMIT -2.08
#define HEADYAW_RIGHT_LIMIT 2.08
#define HEADYAW_ORIGIN		0

ros::Publisher walk_pub;
ros::Publisher head_pub;

void look_around(std::string name, float theta)
{
    /* 
        rostopic pub -1 /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed '{header: auto, joint_names: ["HeadYaw"], joint_angles: [0], speed: 0.1}'
    */
    naoqi_bridge_msgs::JointAnglesWithSpeed msg;
    msg.joint_names.push_back(name);
    msg.joint_angles.push_back(theta);
    msg.speed= 0.01;
    
    if(theta > HEADYAW_LEFT_LIMIT && theta < HEADYAW_RIGHT_LIMIT)		
        head_pub.publish(msg);
}

bool serviceCallback(nao_control::LookAround::Request  &req, nao_control::LookAround::Response &res)
{
    look_around(req.name, req.theta);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "look_around_service");
    ros::NodeHandle n;

    // Publisher for the head rotating
    head_pub=n.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1);

    // Service for looking around
    ros::ServiceServer service = n.advertiseService("lookaround", serviceCallback);
    
    ros::spin();

    return 0;
}