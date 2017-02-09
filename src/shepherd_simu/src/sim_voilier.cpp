#include "ros/ros.h"
#include "std_msgs/String.h"
#include "shepherd_disp/SailboatPose.h"
#include "shepherd_reg/SailboatCmd.h"
#include "shepherd_simu/WorldInfo.h"
#include "sim_voilier.h"
#include "sailboat.h"

// ======================== NODE INIT ===========================

ros::NodeHandle initNode(int argc, char **argv, std::string name){
    // Set up ROS.
    ros::init(argc, argv, name);
    ros::NodeHandle n;

    // You can populate the node with features by looking at http://wiki.ros.org/ROSNodeTutorialC%2B%2B

    // Create a publisher and name the topic.
    pubSailboatPose = n.advertise<shepherd_disp::SailboatPose>("sailboat/pose_real", 10);

    // Create suscribers
    subCmd = n.subscribe("sailboat/cmd", 1000, &cmdCallback);
    subEnv = n.subscribe("world/env", 10, &envCallback);

    return n;
}

// ======================== NODE PROCESS ========================

void cmdCallback(const shepherd_reg::SailboatCmd::ConstPtr& msg)
{
    sailboatCmd.rudder_angle = msg->rudder_angle;
    sailboatCmd.sail_angle = msg->sail_angle;
    ROS_INFO("Boat commands : [%f] [%f]", msg->rudder_angle, msg->sail_angle);
}
void envCallback(const shepherd_simu::WorldInfo::ConstPtr& msg)
{
    worldEnv.wind_angle = msg->wind_angle;
    worldEnv.wind_strength = msg->wind_strength;
    ROS_INFO("World parameters : [%f] [%f]", msg->wind_angle, msg->wind_strength);
}

int main(int argc, char **argv)
{
    ros::NodeHandle n = initNode(argc, argv, "sim_voilier");

    // Objects creation
    double dt = r.expectedCycleTime().sec+r.expectedCycleTime().nsec/1000000000.0;
    printf("dt = %f\n",dt);
    Sailboat boat  = Sailboat(0,0,dt);

    // Server parameter to include here
    boat.setTargetTriangle(100,100);

    // Main loop.
    while (n.ok())
    {
        // Fait avancer la simulation d'un pas
        boat.clock(sailboatCmd.rudder_angle,sailboatCmd.sail_angle);

        sailboatPose.pose.theta = boat.theta;
        sailboatPose.pose.x = boat.x;
        sailboatPose.pose.y = boat.y;

        // Publish the message.
        pubSailboatPose.publish(sailboatPose);

        loop(n);
    }

    return 0;
}