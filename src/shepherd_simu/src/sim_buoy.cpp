#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "sim_buoy.h"
#include  "buoy.h"



ros::NodeHandle initNode(int argc, char **argv, std::string name){

  // Node init
  ros::init(argc,argv,name);
  ros::NodeHandle n;

  // publisher
  pubBuoyPose = n.advertise<geometry_msgs::Point>("buoy/pose_real",1000);

  // subcriber
  subCmd = n.subscribe("buoy/cmd",1000,&cmdCallback);

}

void cmdCallback(const std_msgs::Float64::ConstPtr& msg){

  buoy.setCommand(msg -> data);
  ROS_INFO("BUOY COMMAND: [%f] ",msg -> data);

}

int main(int argc, char **argv){

  ros::NodeHandle n = initNode(argc,argv,"sim_buoy");
  ros::Rate r(100);

  //

  double dt = r.expectedCycleTime().sec+r.expectedCycleTime().nsec/1000000000.0;
  printf("dt = %f\n",dt);

  double accelRate =10;

  buoy = Buoy(10,42,31,45,0,accelRate*dt);


  // Main loop
  while (n.ok()) {

    buoy.clock();

    buoyPose.x = buoy.x;
    buoyPose.y = buoy.y;
    buoyPose.z = buoy.z;

    //Publish
    pubBuoyPose.publish(buoyPose);

    //loop
    ros::spinOnce();
    r.sleep();

  }


  return 0;
}
