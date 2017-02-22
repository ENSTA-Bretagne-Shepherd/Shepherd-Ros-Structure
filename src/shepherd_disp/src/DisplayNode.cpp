#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include "shepherd_disp/SailboatPose.h"
#include <iostream>
#include <GL/glut.h>
#include "networktools.h"
#include "buoy.h"
#include "sailboat.h"
#include "geometry_msgs/Point.h"
#include "shepherd_msg/SailboatPose.h"


char message[1024];
Sailboat0 S1;
Sailboat0 S2;
Sailboat0 S3;
Sailboat0 S4;

Buoy0 B1;
Buoy0 B2;
Buoy0 B3;
Buoy0 B4;
Buoy0 B5;

void chatterCallbackBuoy(const geometry_msgs::Point::ConstPtr& msg,Buoy0 *B)
{

B->x=msg->x;
B->y=msg->y;
B->z=msg->z;

}
void chatterCallbackBuoy1(const geometry_msgs::Point::ConstPtr& msg)
{
chatterCallbackBuoy(msg,&B1);
}
void chatterCallbackBuoy2(const geometry_msgs::Point::ConstPtr& msg)
{
chatterCallbackBuoy(msg,&B2);

}
void chatterCallbackBuoy3(const geometry_msgs::Point::ConstPtr& msg)
{

chatterCallbackBuoy(msg,&B3);
}
void chatterCallbackBuoy4(const geometry_msgs::Point::ConstPtr& msg)
{

chatterCallbackBuoy(msg,&B4);
}
void chatterCallbackBuoy5(const geometry_msgs::Point::ConstPtr& msg)
{

chatterCallbackBuoy(msg,&B5);

}

void chatterCallback1(const shepherd_disp::SailboatPose::ConstPtr& msg,Sailboat0 *S)
{
S->x=(msg->pose).x;
S->y=(msg->pose).y;
S->theta=(msg->pose).theta;

}

void chatterCallback2(const shepherd_disp::SailboatPose::ConstPtr& msg,Sailboat0 *S)
{

S->x=(msg->pose).x;
S->y=(msg->pose).y;
S->theta=(msg->pose).theta;

}

void chatterCallback3(const shepherd_disp::SailboatPose::ConstPtr& msg,Sailboat0 *S)
{

S->x=(msg->pose).x;
S->y=(msg->pose).y;
S->theta=(msg->pose).theta;

}
void chatterCallback4(const shepherd_disp::SailboatPose::ConstPtr& msg,Sailboat0 *S)
{
S->x=(msg->pose).x;
S->y=(msg->pose).y;
S->theta=(msg->pose).theta;		
}




void calllauncher1(const shepherd_disp::SailboatPose::ConstPtr& msg){
chatterCallback1(msg,&S1);
}

void calllauncher2(const shepherd_disp::SailboatPose::ConstPtr& msg){
chatterCallback2(msg,&S2);
}

void calllauncher3(const shepherd_disp::SailboatPose::ConstPtr& msg){
chatterCallback3(msg,&S3);
}

void calllauncher4(const shepherd_disp::SailboatPose::ConstPtr& msg){
chatterCallback4(msg,&S4);
}












int main(int argc, char **argv)
{
//init des variables
 // std::cout << "Node initialization " << std::endl;
  init_connection("3175");
 // char *buffer = wait_connection(message, 5*sizeof(Buoy0) + 4*sizeof(Sailboat0));
  ros::init(argc, argv, "Opengl_Connection");


  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::NodeHandle n3;
  ros::NodeHandle n4;
  ros::NodeHandle b1;
  ros::NodeHandle b2;
  ros::NodeHandle b3;
  ros::NodeHandle b4;
  ros::NodeHandle b5;

  // Subscriber to sailboat pose
  ros::Subscriber sub1 = n1.subscribe("sailboat1/pose_real",1000, calllauncher1);
  ros::Subscriber sub2 = n2.subscribe("sailboat2/pose_real",1000, calllauncher2);
  ros::Subscriber sub3 = n3.subscribe("sailboat3/pose_real",1000, calllauncher3);
  ros::Subscriber sub4 = n4.subscribe("sailboat4/pose_real",1000, calllauncher4);



  ros::Subscriber subb1 = b1.subscribe("buoy1/pose_real",1000, chatterCallbackBuoy1);
  ros::Subscriber subb2 = b2.subscribe("buoy2/pose_real",1000, chatterCallbackBuoy2);
  ros::Subscriber subb3 = b3.subscribe("buoy3/pose_real",1000, chatterCallbackBuoy3);
  ros::Subscriber subb4 = b4.subscribe("buoy4/pose_real",1000, chatterCallbackBuoy4);
  ros::Subscriber subb5 = b5.subscribe("buoy5/pose_real",1000, chatterCallbackBuoy5);
//vérifie si de nouveau messages sont arrivés
//spin est une boucle sans fin qui vérifie si de nouveau messages sont arrivés
//(contrairement à spinOnce qui exécute l'action une seule fois)


//init variables publisher


ros :: Rate loop_rate(1);
while (ros::ok()){
	ros::spinOnce();
	char *output = message;
        ((Sailboat0*)output)[0] = S1;
        ((Sailboat0*)output)[1] = S2;
        ((Sailboat0*)output)[2] = S3;
        ((Sailboat0*)output)[3] = S4;
        output = (char*)(((Sailboat0*)message) + 4);
       
        ((Buoy0*)output)[0] =B1;
        	((Buoy0*)output)[1] =B2;
        	((Buoy0*)output)[2] =B3;
        	((Buoy0*)output)[3] =B4;
        	((Buoy0*)output)[4] =B5; 
     	

	char *buffer = wait_connection(message, 5*sizeof(Buoy0) + 4*sizeof(Sailboat0));
loop_rate.sleep();


}
return 0;
}

