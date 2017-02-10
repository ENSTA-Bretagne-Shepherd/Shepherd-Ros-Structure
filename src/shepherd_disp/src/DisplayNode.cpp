#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include "shepherd_disp/SailboatPose.h"
#include <iostream>
#include <GL/glut.h>
#include "networktools.h"
#include "buoy.h"
#include "sailboat.h"
char message[1024];

void chatterCallback(const shepherd_disp::SailboatPose::ConstPtr& msg)
{
  ROS_INFO("I received a position: ([%f], [%f], [%f])", (msg->pose).x, (msg->pose).y, (msg->pose).theta);

  // send sailboat position to unity
  char *output = message;


Sailboat0 S;
S.x=(msg->pose).x;
S.y=(msg->pose).y;
S.theta=(msg->pose).theta;

        	((Sailboat0*)output)[0] = S;
        	/*((Sailboat0*)output)[1] = env.vec_sailboat[1];
        	((Sailboat0*)output)[2] = env.vec_sailboat[2];
        	((Sailboat0*)output)[3] = env.vec_sailboat[3];
        */
        	output = (char*)(((Sailboat0*)message) + 4);
 /*       
         	((Buoy0*)output)[0] =env.vec_buoy[0];
        	((Buoy0*)output)[1] =env.vec_buoy[1];
        	((Buoy0*)output)[2] =env.vec_buoy[2];
        	((Buoy0*)output)[3] =env.vec_buoy[3];
        	((Buoy0*)output)[4] =env.vec_buoy[4];
*/         	char *buffer = wait_connection(message, 5*sizeof(Buoy0) + 4*sizeof(Sailboat0));
}
int main(int argc, char **argv)
{
//init des variables
 // std::cout << "Node initialization " << std::endl;
  init_connection("3175");
char *buffer = wait_connection(message, 5*sizeof(Buoy0) + 4*sizeof(Sailboat0));
  std::cout << "ça marche" << std::endl;
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
