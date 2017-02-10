
#pragma once
#include "buoy.h"

Buoy buoy(0,0,0,0,0,0);
std_msgs::Float64 u;
ros::Publisher pubBuoyPose;
geometry_msgs::Point buoyPose;

ros::Subscriber subCmd;

void cmdCallback(const std_msgs::Float64::ConstPtr& msg);

ros::NodeHandle initNode(int argc, char **argv, std::string name);

int main(int argc, char **argv);
