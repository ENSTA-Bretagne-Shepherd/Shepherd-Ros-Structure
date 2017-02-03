#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include "shepherd_disp/SailboatPose.h"
#include "shepherd_reg/SailboatCmd.h"

class TriangleController
{
public:
  TriangleController(){
    pose_sub = node.subscribe("sailboat/all", 1, &TriangleController::updatePose, this);
    wind_sub = node.subscribe("env/wind", 1, &TriangleController::updateWind, this);
    center_sub = node.subscribe("sailboat/triangleCenter", 1, &TriangleController::updateCenter, this);

    cmd_pub = node.advertise<shepherd_reg::SailboatCmd>("sailboat/cmd", 1);

  }

  void updatePose(const shepherd_disp::SailboatPose::ConstPtr& msg){
    x = (msg->pose).x;
    y = (msg->pose).y;
    theta = (msg->pose).theta;
    ROS_INFO("I received a position: ([%f], [%f], [%f])", x, y, theta);
  }

  void updateWind(const std_msgs::Float64::ConstPtr& msg){
    wind = msg->data;
    ROS_INFO("I received a wind: [%f]", wind);
  }

  void updateCenter(const std_msgs::Float64MultiArray::ConstPtr& msg){
    cx = msg->data[0];
    cy = msg->data[1];
    ROS_INFO("I received a center: ([%f], [%f])", cx, cy);
  }

  void updateCommand(){
    cmd.rudder_angle = 10;
    cmd.sail_angle = 60;
  }

  void spin(){

    ros::Rate loop(10);

    while (ros::ok()){

      // call all waiting callbacks
      ros::spinOnce();
      
      updateCommand();
      // publish the command
      cmd_pub.publish(cmd);

      loop.sleep();

    }
  }

private:
  // Node
  ros::NodeHandle node;
  // 
  ros::Subscriber pose_sub;
  ros::Subscriber wind_sub;
  ros::Subscriber center_sub; // triangleCenter
  ros::Publisher cmd_pub;

  float x, y, theta;
  float wind;
  float cx, cy;

  shepherd_reg::SailboatCmd cmd;

  
};


int main(int argc, char **argv)
{
  // Node initialization
  std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "regulation_voilier");

  TriangleController controller;

  controller.spin();

  return 0;
}
