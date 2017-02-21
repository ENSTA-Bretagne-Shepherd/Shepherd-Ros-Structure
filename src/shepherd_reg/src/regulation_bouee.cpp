#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <math.h>
#include <shepherd_msg/WorldInfo.h>

//#include <message_event.h>

class BuoySimpleController
{
public:
  BuoySimpleController(){
    pose_sub = node.subscribe("buoy/pose_est", 1, &BuoySimpleController::updatePose, this);

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


/*

 */
class BuoyBetterController
{
public:
  BuoyBetterController(){
    pose_sub = node.subscribe("buoy/pose_est", 1, &BuoyBetterController::updatePose, this);
//name :simu2loc type msg:Msg_Simu2Loc
    cmd_pub = node.advertise<std_msgs::Float64>("buoy/cmd", 1);
    vertSpeed = 1;
    lastTime = ros::Time::now().toSec();
  }


  void updatePose(const geometry_msgs::Point::ConstPtr& msg)
  {
    double t  = ros::Time::now().toSec();
    double dt = t-lastTime;
    lastTime  = t;
    //if(x == NULL)
    if(x == 0)
    {
      x  = msg->x;
      y  = msg->y;
      z  = msg->z;
      xd = 0;
      yd = 0;
      uc = 0;
      vc = 0;
    }
    xl  = x;
    yl  = y;
    x   = msg->x;
    y   = msg->y;
    z   = msg->z;
    ROS_INFO("I received an estimated position: ([%f], [%f], [%f])", x, y, z);
    xdl = xd;
    ydl = yd;
    xd  = (x-xl)/dt;
    yd  = (y-yl)/dt;
    uc  = (xd-xdl)/dt;
    vc  = (yd-ydl)/dt;
  }

  void updateCommand(){



    if(z<10 && vertSpeed==-1)
    {
        vertSpeed = 1;
    }
    else if(z>500 && vertSpeed==1)
    {
        vertSpeed = -1;
    }
    u.data = vertSpeed*u.data;
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
  //Msg_Simu2Loc

  float x, y, z;//! coordonnees du point courant
  float xg, yg;//!coordonnees du centre de gravite des voiliers
  float xl, yl;//!coordonnees du point precedant
  float xd, yd;//!vitesse estimee au point courant
  float xdl, ydl;//!vitesse estimee au point precedent
  float uc, vc;//!composante estimee du courant au point (x, y, z)(est egal a lacceleration estime)
  float vertSpeed;
  double lastTime;
  std_msgs::Float64 u;
};




int main(int argc, char **argv)
{
  // Node initialization
  std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "regulation_bouee");

  BuoySimpleController controller;

  controller.spin();
  return 0;
}
