//
// Created by tag on 16/12/16.
//

#include "world.h"
#include "sailboat.h"
#include "buoy.h"
#include "config.h"

World::World(int sailboatNb, int buoyNb) {

    // Boat creation
    for (int i = 0; i < sailboatNb; ++i) {
        const double x = 0;
        const double y = 0;
        vec_sailboat.push_back(Sailboat(x,y));
    }

    // Buoy creation
    for (int i = 0; i < buoyNb; ++i) {
        //vec_buoy.push_back(Buoy(

        //));
        double x = 0;
        double y = 0;
        double z = 0;
        const double u = 0;
        vec_buoy.push_back(Buoy(i,x,y,z,u));
    }
}

void World::initialize()
{
    const double tabSailX[] = X_BOAT_TARGET;
    const double tabSailY[] = Y_BOAT_TARGET;

    for (int i = 0; i < vec_sailboat.size(); ++i) {
        vec_sailboat[i].setTargetTriangle(tabSailX[i],tabSailY[i]);
    }
}

void World::clock()
{
    for (auto &&sailboat : vec_sailboat) {
        sailboat.clock();
    }

    for (auto &&buoy : vec_buoy) {
        buoy.clock();
    }
}
