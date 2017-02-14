#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include "shepherd_msg/SailboatPose.h"
#include "shepherd_msg/SailboatCmd.h"
#include <math.h>
#include <shepherd_msg/WorldInfo.h>

class TriangleController
{
public:
  TriangleController(){
    pose_sub = node.subscribe("pose_est", 1, &TriangleController::updatePose, this);
    wind_sub = node.subscribe("/world/env", 1, &TriangleController::updateWind, this);
    center_sub = node.subscribe("triangleCenter", 1, &TriangleController::updateCenter, this);

    cmd_pub = node.advertise<shepherd_msg::SailboatCmd>("cmd", 1);

    iseg = 0; q = 1;
  }

  void updatePose(const shepherd_msg::SailboatPose::ConstPtr& msg){
    x = (msg->pose).x;
    y = (msg->pose).y;
    theta = (msg->pose).theta;
    ROS_INFO("I received a position: ([%f], [%f], [%f])", x, y, theta);
  }

  void updateWind(const shepherd_msg::WorldInfo::ConstPtr& msg){
    wind = msg->wind_angle;
    ROS_INFO("I received a wind: [%f]", wind);
  }

  void updateCenter(const std_msgs::Float64MultiArray::ConstPtr& msg){
    cx = msg->data[0];
    cy = msg->data[1];
    ROS_INFO("I received a center: ([%f], [%f])", cx, cy);
  }

  void updateCommand(){
    double r=10;
    double zeta=M_PI/4;
    double ax,bx,ay,by;
    ax = cx + 50 * cos(iseg * 2 * M_PI/3);
    ay = cy + 50 * sin(iseg * 2 * M_PI/3);
    bx = cx + 50 * cos((iseg + 1) * 2 * M_PI/3);
    by = cy + 50 * sin((iseg + 1) * 2 * M_PI/3);
    if((x-bx)*(ax-bx) + (y-by)*(ay-by) < 0)iseg++;
    
    double e = ((bx-ax)*(y-ay)-(x-ax)*(by-ay))/hypot(ax-bx,ay-by);
    if (fabs(e)>r) q=0;  //The robot is now free from its closed-hauled mode
    double phi=atan2(by-ay,bx-ax);
    double thetabar=phi-0.5*atan(e/r);
    if ((q==0)&((cos(wind-thetabar)+cos(zeta)<0)|((fabs(e)<r)&(cos(wind-phi)+cos(zeta)<0)))) q=static_cast<int>(sign(e));
    if (q!=0)  thetabar=M_PI+wind-zeta*q;
    double dtheta=theta-thetabar;

    // command
    double deltag, deltavmax;
    deltag=(1/M_PI)*(atan(tan(0.5*dtheta)));
    deltavmax=0.5*M_PI*(0.5*(cos(wind-thetabar)+1));
    // Update the command message
    cmd.rudder_angle = deltag*2;
    cmd.sail_angle = deltavmax;
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

  // variables obscure de Dyson
  int iseg, q;

  shepherd_msg::SailboatCmd cmd;

  // helper methods
  double sign(double a){
    if (a > 0) return 1; else return -1;
  }

  
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
