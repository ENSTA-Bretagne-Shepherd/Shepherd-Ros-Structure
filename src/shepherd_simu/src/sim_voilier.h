//
// Created by tag on 03/02/17.
//

#pragma once

#include <sailboat.h>

// Global variables
/**
 * Initialisation du bateau
 */
Sailboat boat(0,0,0);

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
void cmdCallback(const shepherd_reg::SailboatCmd::ConstPtr& msg);

/**
 * SUBSCRIBER
 * Variables d'environnement
 */
ros::Subscriber subEnv;
shepherd_simu::WorldInfo worldEnv;
void envCallback(const shepherd_simu::WorldInfo::ConstPtr& msg);

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
 * Node sim_voilier
 * Main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv);

