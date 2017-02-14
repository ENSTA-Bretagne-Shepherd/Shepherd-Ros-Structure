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

    // subscribe aux ping des voiliers
    voiliers_sub1 = node.subscribe("sailboat1/ping", 1, &BuoyBetterController::updateVoilier, this);
    voiliers_sub2 = node.subscribe("sailboat2/ping", 1, &BuoyBetterController::updateVoilier, this);
    voiliers_sub3 = node.subscribe("sailboat3/ping", 1, &BuoyBetterController::updateVoilier, this);
    voiliers_sub4 = node.subscribe("sailboat4/ping", 1, &BuoyBetterController::updateVoilier, this);

    cmd_pub = node.advertise<std_msgs::Float64>("buoy/cmd", 1);
    vertSpeed = 1;
    lastTime = ros::Time::now().toSec();
  }


  void updatePose(const geometry_msgs::Point::ConstPtr& msg)
  {
    double t  = ros::Time::now().toSec();
    double dt = t-lastTime;
    lastTime  = t;
    if(x == NULL)
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

  void updateVoilier(const msg::ping::ConstPtr& msg){
	switch(msg->id){
	case 1:
		xmin1=msg->xmin;
		xmax1=msg->xmax;
		ymin1=msg->ymin;
		ymax1=msg->ymax;
		break;
	case 2:
		xmin2=msg->xmin;
		xmax2=msg->xmax;
		ymin2=msg->ymin;
		ymax2=msg->ymax;
		break;
	case 3:
		xmin3=msg->xmin;
		xmax3=msg->xmax;
		ymin3=msg->ymin;
		ymax3=msg->ymax;
		break;
	case 4:
		xmin4=msg->xmin;
		xmax4=msg->xmax;
		ymin4=msg->ymin;
		ymax4=msg->ymax;
		break;
	}
  }

  void updateCommand(){

    // calcul du barycentre des 4 voiliers
    xBar=( (xMax1+xMin1)/2 + (xMax2+xMin2)/2 + (xMax3+xMin3)/2 + (xMax4+xMin4)/2 )/4;
    yBar=( (yMax1+yMin1)/2 + (yMax2+yMin2)/2 + (yMax3+yMin3)/2 + (yMax4+yMin4)/2 )/4;

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

  // inscription aux positions des voiliers
  ros::Subscriber voiliers_sub1;
  ros::Subscriber voiliers_sub2;
  ros::Subscriber voiliers_sub3;
  ros::Subscriber voiliers_sub4; 

  // coordonnees des 4 voiliers
  float xmin1, xmax1, ymin1, ymax1;
  float xmin2, xmax2, ymin2, ymax2;
  float xmin3, xmax3, ymin3, ymax3;
  float xmin4, xmax4, ymin4, ymax4;

  // coordonnees du barycentre
  float xBar,yBar;

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
