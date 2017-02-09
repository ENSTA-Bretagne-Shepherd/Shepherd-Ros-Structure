//
// Created by tag on 03/02/17.
//

#pragma once

// Global variables

// Node init
ros::Rate r(10);

// Publishers and subscribers
/**
 * PUBLISHER
 * Position du bateau
 */
ros::Publisher pubSailboatPose;
shepherd_disp::SailboatPose sailboatPose;

/**
 * SUBSCRIBER
 * Commande du bateau
 */
ros::Subscriber subCmd;
shepherd_reg::SailboatCmd sailboatCmd;

/**
 * SUBSCRIBER
 * Variables d'environnement
 */
ros::Subscriber subEnv;
shepherd_simu::WorldInfo worldEnv;

/**
 * Initialise la node
 *
 * Publish :
 *  TOPIC : sailboatX/pose_real
 * Subscribe :
 *  TOPIC : sailboatX/cmd
 *  TOPIC : world/env
 *
 * @param argc
 * @param argv
 * @param name
 * @return
 */
ros::NodeHandle initNode(int argc, char **argv, std::string name);

/**
 * Synchronise la node avec le temps de ROS
 * Fonction incluse dans le header car elle n'a pas besoin d'être modifiée.
 * @param n
 */
void loop(ros::NodeHandle n){
    // ROS node management methods
    ros::spinOnce();
    r.sleep();
}

/**
 * Node sim_voilier
 * Main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv);

