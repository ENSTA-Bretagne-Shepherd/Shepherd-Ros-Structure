//
// Created by tag on 09/02/17.
//

#include <shepherd_simu/WorldInfo.h>
#include "ros/ros.h"
#include "sim_world.h"

// ======================== NODE INIT ===========================

ros::NodeHandle initNode(int argc, char **argv, std::string name){
    // Set up ROS.
    ros::init(argc, argv, name);
    ros::NodeHandle n;

    // You can populate the node with features by looking at http://wiki.ros.org/ROSNodeTutorialC%2B%2B

    // Create a publisher and name the topic.
    pubWorldEnv = n.advertise<shepherd_simu::WorldInfo>("world/env", 1);

    return n;
}

// ======================== NODE PROCESS ========================

int main(int argc, char **argv)
{
    ros::NodeHandle n = initNode(argc, argv, "sim_world");
    ros::Rate r(1);

    // Server parameter to include here
    worldInfo.wind_strength = 3; // m/s/s ??
    worldInfo.wind_angle = -M_PI/2; // rad sens horaire à partir du nord //Le vent vient de l'ouest

    // Main loop.
    while (n.ok())
    {
        // Publish the message.
        pubWorldEnv.publish(worldInfo);

        // Loop
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}