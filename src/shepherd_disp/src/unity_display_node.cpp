#include "ros/ros.h"
#include <iostream>
#include "unity_api/unityapi.cpp"
#include "shepherd_disp/SailboatPose.h"

class SailboatSubscriber
{
public:
  SailboatSubscriber(ros::NodeHandle n, std::string topic_name, std::string sailboat_name){
    this->topic_name = topic_name;
    this->sailboat_name = sailboat_name;
    pose_sub = n.subscribe(topic_name, 1000, &SailboatSubscriber::sendPoseToUnity, this);
  }

  void sendPoseToUnity(const shepherd_disp::SailboatPose::ConstPtr& msg){
     ROS_INFO("I received a position: ([%f], [%f], [%f]) for [%s]", (msg->pose).x, (msg->pose).y, (msg->pose).theta, sailboat_name.c_str());

    // send sailboat position to unity
    sendSailBoatState(sailboat_name, (msg->pose).x, (msg->pose).y, (msg->pose).theta, msg->sail_angle);
  }

private:
  std::string topic_name;
  std::string sailboat_name;
  ros::Subscriber pose_sub;
  
};

int main(int argc, char **argv)
{
  // Node initialization
  std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "unity_connection");

  // Connection to unity
  std::cout << "Trying to connect to " << argv[1] << std::endl;
  init_unity_connection(argv[1], 13000);
  
  ros::NodeHandle n;

  // Subscriber to all 4 sailboat poses
  SailboatSubscriber sail1_sub(n, "sailboat1/pose_real", "sailboat1");
  SailboatSubscriber sail2_sub(n, "sailboat2/pose_real", "sailboat2");
  SailboatSubscriber sail3_sub(n, "sailboat3/pose_real", "sailboat3");
  SailboatSubscriber sail4_sub(n, "sailboat4/pose_real", "sailboat4");

  // spin
  ros::spin();

  return 0;
}
