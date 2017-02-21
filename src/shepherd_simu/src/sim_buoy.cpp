#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "sim_buoy.h"
#include "buoy.h"
#include <stdlib.h>



ros::NodeHandle initNode(int argc, char **argv, std::string name){

  // Node init
  ros::init(argc,argv,name);
  ros::NodeHandle n;

  // publisher
  pubBuoyPose = n.advertise<geometry_msgs::Point>("buoy/pose_real",1000);
  pubBuoyDepth = n.advertise<std_msgs::Float64>("buoy/depthNoisy",1000);

  // subcriber
  subCmd = n.subscribe("buoy/cmd",1000,&cmdCallback);

}

void cmdCallback(const std_msgs::Float64::ConstPtr& msg){

  buoy.setCommand(msg -> data);
  ROS_INFO("BUOY COMMAND: [%f] ",msg -> data);

}

double rand01(){
    return ((double) rand() / (RAND_MAX));
}

double fRand(double fMin, double fMax){
    double r = rand01();
    return fMin + rand01() * (fMax - fMin);
}


int main(int argc, char **argv){

  ros::NodeHandle n = initNode(argc,argv,"sim_buoy");
  printf("petit test izi\n");
  ros::Rate r(100);
  printf("petit test izi 1\n");

  //publisher
  printf("petit test izi 2\n");

  double dt = r.expectedCycleTime().sec+r.expectedCycleTime().nsec/1000000000.0;
  printf("dt = %f\n",dt);

  double accelRate =10;

  buoy = Buoy(10,42,31,45,0,accelRate*dt);


    // Get parameters of the simulation
    double depth_noise;
    ros::param::param<double>("buoy_depth_sensor_noise", depth_noise, 0.5);
    ROS_INFO("Depth noise : [%f]", depth_noise);


  // Main loop
  while (n.ok()) {

    buoy.clock();

    buoyPose.x = buoy.x;
    buoyPose.y = buoy.y;
    buoyPose.z = buoy.z;
    buoyDepthNoisy.data = fRand(buoy.z-depth_noise,buoy.z+depth_noise);

    //Publish
    pubBuoyPose.publish(buoyPose);
    pubBuoyDepth.publish(buoyDepthNoisy);

    //loop
    ros::spinOnce();
    r.sleep();

  }


  return 0;
}
