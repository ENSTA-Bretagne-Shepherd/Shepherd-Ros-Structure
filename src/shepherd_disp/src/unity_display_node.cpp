#include "ros/ros.h"
#include <iostream>
#include "unity_api/unityapi.cpp"
#include "shepherd_disp/SailboatPose.h"
#include "geometry_msgs/Point.h"
#include <sstream>

class SailboatSubscriber
{
public:
    SailboatSubscriber(ros::NodeHandle n, std::string topic_name, std::string sailboat_name){
        this->topic_name = topic_name;
        this->sailboat_name = sailboat_name;
        pose_sub = n.subscribe(topic_name, 1000, &SailboatSubscriber::sendPoseToUnity, this);
    }

    void sendPoseToUnity(const shepherd_disp::SailboatPose::ConstPtr& msg){
        ROS_INFO("I received a sailboat position: ([%f], [%f], [%f]) for [%s]", (msg->pose).x, (msg->pose).y, (msg->pose).theta, sailboat_name.c_str());

        // send sailboat position to unity
        sendSailBoatState(sailboat_name, (msg->pose).x, (msg->pose).y, (msg->pose).theta, msg->sail_angle);
    }

private:
    std::string topic_name;
    std::string sailboat_name;
    ros::Subscriber pose_sub;

};

class BuoySubscriber
{
public:
    BuoySubscriber(ros::NodeHandle n, std::string topic_name, std::string buoy_name){
        this->topic_name = topic_name;
        this->buoy_name = buoy_name;
        pose_sub = n.subscribe(topic_name, 1000, &BuoySubscriber::sendPoseToUnity, this);
        ROS_INFO("Subscribed to %s", topic_name.c_str());
    }

    void sendPoseToUnity(const geometry_msgs::Point::ConstPtr& msg){
        ROS_INFO("I received a buoy position: ([%f], [%f], [%f])", (msg->x), (msg->y), (msg->z));

        // send sailboat position to unity
        sendBuoyState(buoy_name, msg->x, msg->y, msg->z);
    }

private:
    std::string topic_name;
    std::string buoy_name;
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

    // Subscriber to all buoy poses
    int nbuoy = 1;
    std::vector<BuoySubscriber> buoySubscribers;
    for (int i = 0; i < nbuoy; ++i) {
        std::string buoy_name = "buoy" + std::to_string(i);
        std::cout << "Buoy name:" << buoy_name << std::endl;
        std::string topic_name = buoy_name + "/pose_real";
        BuoySubscriber buoyX_sub(n, topic_name, buoy_name);
        buoySubscribers.push_back(buoyX_sub);
    }


    // spin
    ros::spin();

    return 0;
}
