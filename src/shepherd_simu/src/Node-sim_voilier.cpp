#include "ros/ros.h"
#include "std_msgs/String.h"
#include "shepherd_disp/SailboatPose.h"
#include "shepherd_reg/SailboatCmd.h"
#include "Node-sim_voilier.h"

#include "config.h"
#include "sailboat.h"

// ______________________________________________________________
// ======================== NODE INIT ===========================

RosNode::RosNode(double rate):r(rate){

    // Create a publisher and name the topic.
    pubSailboatPose = advertise<shepherd_disp::SailboatPose>("pose_real", 10);

    // Create suscribers
    subCmd = subscribe("sailboat/cmd", 1000, &cmdCallback);
}

void RosNode::loop(){
    // ROS node management methods
    ros::spinOnce();
    r.sleep();
}

// ______________________________________________________________
// ======================== NODE PROCESS ========================

void RosNode::cmdCallback(const shepherd_reg::SailboatCmd::ConstPtr& msg)
{


    ROS_INFO("I heard: [%f] [%f]", msg->rudder_angle, msg->sail_angle);
    // TODO : update command for the simulation

}

/**
 * Node sim_voilier
 *
 * Publish :
 *  TOPIC : sailboatX/pose_real
 * Subscribe :
 *  TOPIC : sailboatX/cmd
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "sim_voilier");
    RosNode n = RosNode(10);

    // Objects creation
    double dt = n.r.expectedCycleTime().sec+n.r.expectedCycleTime().nsec/1000000000.0;
    printf("dt = %f\n",dt);
    Sailboat boat  = Sailboat(0,0,dt);

    // Server parameter to include here
    boat.setTargetTriangle(100,100);

    // Main loop.
    while (n.ok())
    {
        // C'est la node pour la simulation physique d'un seul bateau. DOnc il faut shunter la classe World
        // Du coup on perd le temps de simulation mais je pense que c'est pas trop grave


        boat.clock(n.sailboatCmd.rudder_angle,n.sailboatCmd.sail_angle);

        //Temps interne
        //int dt = 10; //sec
        //boat.simuTime += dt;

        n.sailboatPose.pose.theta = boat.theta;
        n.sailboatPose.pose.x = boat.x;
        n.sailboatPose.pose.y = boat.y;

        // Publish the message.
        n.pubSailboatPose.publish(n.sailboatPose);

        n.loop();
    }

    return 0;
} // end main()

/*
 * TRASH
 */