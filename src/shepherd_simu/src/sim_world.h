//
// Created by tag on 09/02/17.
//

#pragma once

// Global variables

// Publishers and subscribers
/**
 * PUBLISHER
 * Etat de l'environnement
 */
ros::Publisher pubWorldEnv;
shepherd_simu::WorldInfo worldInfo;


/**
 * Initialise la node
 *
 * Publish :
 *  TOPIC : world/env
 *
 * @param argc
 * @param argv
 * @param name
 * @return
 */
ros::NodeHandle initNode(int argc, char **argv, std::string name);

/**
 * Node sim_world
 * Main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv);