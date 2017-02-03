#include "ros/ros.h"
#include "std_msgs/String.h"
#include "shepherd_disp/SailboatPose.h"
#include "Node-sim_voilier.h"

#include "config.h"
#include "world.h"

// ______________________________________________________________
// ======================== NODE INIT ===========================

RosNode::RosNode(std::string name,double rate, int argc, char **argv):r(rate){
    // Set up ROS.
    ros::init(argc, argv, name);

    // Create a new NodeExample object.
//    NodeExample *node_example = new NodeExample();

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
//    dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
//    dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
//    cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
//    dr_srv.setCallback(cb);

    /* Initialize node parameters from launch file or command line.
   Use a private node handle so that multiple instances of the node can
   be run simultaneously while using different parameters.
   Parameters defined in the .cfg file do not need to be initialized here
   as the dynamic_reconfigure::Server does this for you.*/
//    ros::NodeHandle private_node_handle_("~");
//    private_node_handle_.param("rate", rate, int(40));
//    private_node_handle_.param("topic", topic, string("example"));


    // Create a publisher and name the topic.
    pubSailboatPose = advertise<shepherd_disp::SailboatPose>("pose_real", 10);

    // Create suscribers

    subCmd = subscribe("cmd", 1000, cmdCallback);
}

void RosNode::loop(){
    // ROS node management methods
    ros::spinOnce();
    r.sleep();
}

// ______________________________________________________________
// ======================== NODE PROCESS ========================

void RosNode::cmdCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());

}

/**
 * Node sim_voilier
 *
 * Publish :
 *  TOPIC : sailboatX/pose_real
 * Suscribe :
 *  TOPIC : sailboatX/cmd
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
    RosNode n = RosNode("sim_voilier", 10, argc,argv);

    // Objects creation
    World env = World(BOAT_NUMBER,BUOY_NUMBER);
    env.initialize();

    // Main loop.
    while (n.ok())
    {

        env.clock();

        //Temps interne
        int dt = 10; //sec
        env.simuTime += dt;

        n.sailboatPose.pose.theta = env.vec_sailboat[0].theta;
        n.sailboatPose.pose.x = env.vec_sailboat[0].x;
        n.sailboatPose.pose.y = env.vec_sailboat[0].y;

        // Publish the message.
        n.pubSailboatPose.publish(n.sailboatPose);

        n.loop();
    }

    return 0;
} // end main()

/*
 * TRASH
 */