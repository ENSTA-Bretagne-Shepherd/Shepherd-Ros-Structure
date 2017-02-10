#include "ros/ros.h"
#include "std_msgs/String.h"
#include "shepherd_msg/SailboatPose.h"
#include "shepherd_msg/SailboatCmd.h"
#include "shepherd_msg/WorldInfo.h"
#include "sim_voilier.h"
#include "sailboat.h"

// ======================== NODE INIT ===========================

/**
 * Initialise la node
 *
 * Publish :
 *  TOPIC : sailboatX/pose_real
 * Subscribe :
 *  TOPIC : sailboatX/cmd
 *  TOPIC : world/env
 *
 * @param argc
 * @param argv
 * @param name
 * @return
 */
ros::NodeHandle initNode(int argc, char **argv, std::string name){
    // Set up ROS.
    ros::init(argc, argv, name);
    ros::NodeHandle n;

    // You can populate the node with features by looking at http://wiki.ros.org/ROSNodeTutorialC%2B%2B

    // Create a publisher and name the topic.
    pubSailboatPose = n.advertise<shepherd_msg::SailboatPose>("sailboat/pose_real", 100);

    // Create suscribers
    subCmd = n.subscribe("sailboat/cmd", 1000, &cmdCallback);
    subEnv = n.subscribe("world/env", 10, &envCallback);

    return n;
}

// ======================== NODE PROCESS ========================

void cmdCallback(const shepherd_msg::SailboatCmd::ConstPtr& msg)
{
    sailboatCmd.rudder_angle = msg->rudder_angle;
    sailboatCmd.sail_angle = msg->sail_angle;
    ROS_INFO("Boat commands : [%f] [%f]", msg->rudder_angle, msg->sail_angle);
}
void envCallback(const shepherd_msg::WorldInfo::ConstPtr& msg)
{
//    worldEnv.wind_angle = msg->wind_angle;
//    worldEnv.wind_strength = msg->wind_strength;
    boat.setWindAccel(msg->wind_strength);
    boat.setWindDir(msg->wind_angle);
    ROS_INFO("World parameters : [%f] [%f]", msg->wind_angle, msg->wind_strength);
}

int main(int argc, char **argv)
{
    ros::NodeHandle n = initNode(argc, argv, "sim_voilier");
    ros::Rate r(100);

    // Objects creation
    double dt = r.expectedCycleTime().sec+r.expectedCycleTime().nsec/1000000000.0;
    printf("dt = %f\n",dt);

    double accelRate = 1; // Pour accélérer la simulation (le bateau sera donc aussi commandé plus lentement)
    boat  = Sailboat(0,0,dt*accelRate);

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

        // Loop
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}