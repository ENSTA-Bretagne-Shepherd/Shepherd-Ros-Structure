#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include "shepherd_disp/SailboatPose.h"
#include <iostream>


void chatterCallback(const shepherd_disp::SailboatPose::ConstPtr& msg)
{
  ROS_INFO("I received a position: ([%f], [%f], [%f])", (msg->pose).x, (msg->pose).y, (msg->pose).theta);

  // send sailboat position to unity
  //(*displayptr).sendSailBoatState("auv1", (msg->pose).x, (msg->pose).y, (msg->pose).theta);
}
int main(int argc, char **argv)
{
//init des variables
 // std::cout << "Node initialization " << std::endl;
  ros::init(argc, argv, "Opengl_Connection");


  ros::NodeHandle n;

  // Subscriber to sailboat pose
  ros::Subscriber sub = n.subscribe("sailboat/all", 1000, chatterCallback);


//vérifie si de nouveau messages sont arrivés
//spin est une boucle sans fin qui vérifie si de nouveau messages sont arrivés
//(contrairement à spinOnce qui exécute l'action une seule fois)
ros::spinOnce();


//init variables publisher


ros :: Rate loop_rate(10);
while (true){

}
return 0;
}

