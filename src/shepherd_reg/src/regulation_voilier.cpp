#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "shepherd_disp/SailboatPose.h"
#include "shepherd_reg/SailboatCmd.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const shepherd_disp::SailboatPose::ConstPtr& msg)
{
  ROS_INFO("I received a position: ([%f], [%f], [%f])", (msg->pose).x, (msg->pose).y, (msg->pose).theta);

  // send sailboat position to unity
  // (*displayptr).sendSailBoatState("auv1", (msg->pose).x, (msg->pose).y, (msg->pose).theta);
}

int main(int argc, char **argv)
{
  // Node initialization
  std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "regulation_voilier");

  ros::NodeHandle n;

  // Subscriber to sailboat pose
  ros::Subscriber sub = n.subscribe("sailboat/all", 1000, chatterCallback);
  // Wind:
  double wind = 0;

  // Publisher of the command
  ros::Publisher pub = n.advertise<shepherd_reg::SailboatCmd>("sailboat/cmd", 1000);

  // Publish Frequency
  ros::Rate loop_rate(10);

  // spin
  ros::spin();

  return 0;
}
