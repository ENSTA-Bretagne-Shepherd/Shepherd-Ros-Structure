
#pragma once
#include "buoy.h"

Buoy buoy(0,0,0,0,0,0);
std_msgs::Float64 u;
ros::Publisher pubBuoyPose;
ros::Publisher pubBuoyDepth;
geometry_msgs::Point buoyPose;
std_msgs::Float64 buoyDepthNoisy;

ros::Subscriber subCmd;

void cmdCallback(const std_msgs::Float64::ConstPtr& msg);

ros::NodeHandle initNode(int argc, char **argv, std::string name);

int main(int argc, char **argv);
