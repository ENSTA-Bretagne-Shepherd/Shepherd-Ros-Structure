#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "sim_buoy.h"
#include  "buoy.h"



ros::NodeHandle initNode(int argc, char **argv, std::string name){

  // Node init
  ros::init(argc,argv,name)
  ros::NodeHandle n;

  // publisher
  pubBuoyPose = n.advertise<geometry_msgs::Pose>("buoy/pose_real",100);

  // subcriber
  subCmd = n.subcribe("buoy/cmd",1000,&cmdCallback);

}

void cmdCallback(const std_msgs::Float64::ConstPtr& msg){

  Float64.data = msg -> data;
  ROS_INFO("BUOY COMMAND: [%f] ",msg -> data);

}

int int main(int argc, char const *argv[]) {

  ros:NodeHandle n = initNode(argc,argv,"sim_buoy");
  ros::Rate r(100);

  // Intances
  buoy = Buoy(0,0,0,0,0,0)
  // Main loop
  while (n.ok()) {

    buoy.clock()

    geometry_msgs.Point.x = buoy.x;
    geometry_msgs.Point.y = buoy.y;
    geometry_msgs.Point.z = buoy.z;

    //Publish
    pubBuoyPose.publish(geometry_msgs.Pose)

    //loop
    ros::spinOnce();
    r.sleep

  }


  return 0;
}
