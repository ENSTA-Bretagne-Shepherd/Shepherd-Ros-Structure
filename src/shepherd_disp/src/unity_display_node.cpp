#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "unity_api/unityapi.cpp"
#include "shepherd_msg/SailboatPose.h"

// the connection to unity api must be in a global scope
// DisplayAPI* displayptr;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const shepherd_msg::SailboatPose::ConstPtr& msg)
{
  ROS_INFO("I received a position: ([%f], [%f], [%f])", (msg->pose).x, (msg->pose).y, (msg->pose).theta);

  // send sailboat position to unity
  sendSailBoatState("auv1", (msg->pose).x, (msg->pose).y, (msg->pose).theta, msg->sail_angle);
}

int main(int argc, char **argv)
{
  // Node initialization
  std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "unity_connection");

  ros::NodeHandle n;

  // Subscriber to sailboat pose
  ros::Subscriber sub = n.subscribe("sailboat/pose_real", 1000, chatterCallback);

  // Connection to unity
  std::cout << "Trying to connect to " << argv[1] << std::endl;
  init_unity_connection(argv[1], 13000);
  // displayptr = new DisplayAPI(argv[1], 13000);
  sendSailBoatState("auv1", 0, 0, 0.0, 0.0);

  // spin
  ros::spin();

  return 0;
}
