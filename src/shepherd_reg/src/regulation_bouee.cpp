#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <math.h>
#include <shepherd_msg/WorldInfo.h>

class BuoyController
{
public:
  BuoyController(){
    pose_sub = node.subscribe("buoy/pose_est", 1, &BuoyController::updatePose, this);

    cmd_pub = node.advertise<std_msgs::Float64>("buoy/cmd", 1);
    u.data = 1;
  }

  void updatePose(const geometry_msgs::Point::ConstPtr& msg){
    x = msg->x;
    y = msg->y;
    z = msg->z;
    ROS_INFO("I received an estimated position: ([%f], [%f], [%f])", x, y, z);
  }

  void updateCommand(){

    if(z<10)
    {
        u.data = 1;
    }
    else if(z>500)
    {
        u.data = -1;
    }
  }

  void spin(){

    ros::Rate loop(10);

    while (ros::ok()){

      // call all waiting callbacks
      ros::spinOnce();

      updateCommand();
      // publish the command
      cmd_pub.publish(u);

      loop.sleep();

    }
  }

private:
  // Node
  ros::NodeHandle node;
  //
  ros::Subscriber pose_sub;
  ros::Publisher cmd_pub;

  float x, y, z;
  std_msgs::Float64 u;
};


int main(int argc, char **argv)
{
  // Node initialization
  std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "regulation_bouee");

  BuoyController controller;

  controller.spin();

  return 0;
}
