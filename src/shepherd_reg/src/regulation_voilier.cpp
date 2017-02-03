#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "shepherd_disp/SailboatPose.h"

// the connection to unity api must be in a global scope
DisplayAPI* displayptr;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const shepherd_disp::SailboatPose::ConstPtr& msg)
{
  ROS_INFO("I received a position: ([%f], [%f], [%f])", (msg->pose).x, (msg->pose).y, (msg->pose).theta);

  // send sailboat position to unity
  (*displayptr).sendSailBoatState("auv1", (msg->pose).x, (msg->pose).y, (msg->pose).theta);
}

int main(int argc, char **argv)
{
  // Node initialization
  std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "unity_connection");

  ros::NodeHandle n;

  // Subscriber to sailboat pose
  ros::Subscriber sub = n.subscribe("sailboat/all", 1000, chatterCallback);

  // Connection to unity
  std::cout << "Trying to connect to " << argv[1] << std::endl;
  displayptr = new DisplayAPI(argv[1], 13000);
  (*displayptr).sendSailBoatState("auv1", 0, 0, 0.0);

  // spin
  ros::spin();

  return 0;
}
