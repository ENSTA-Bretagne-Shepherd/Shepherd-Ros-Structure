//
// Created by tag on 03/02/17.
//

#pragma once
/**
 * Classe du noeud sim_voilier
 * Sert de template pour l'instant
 */
class RosNode: public ros::NodeHandle{
private:
public:
    // Tell ros how fast to run this node
    ros::Rate r;

    // publish
    shepherd_disp::SailboatPose sailboatPose;
    ros::Publisher pubSailboatPose;
    // suscribe
    ros::Subscriber subCmd;

    RosNode(std::string name,double rate, int argc, char **argv);
    void loop();

    static void cmdCallback(const std_msgs::String::ConstPtr& msg);
};
