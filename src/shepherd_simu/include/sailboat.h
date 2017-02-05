#ifndef sailboat_H
#define sailboat_H

#include <math.h>

#include <GL/gl.h>
#include <GL/glu.h>

#pragma once

/**
 * Classe bu bateau
 */
class Sailboat
{    public:
            double dt;
            double fg,fv,gamma,deltav; //link variables
//            double deltag,deltavmax; //controls

            // Variables d'etat
            double x; //!state variable
            double y; //!state variable
            double theta; //!state variable
            double v; //!state variable
            double omega; //!state variable
            double phi; //!state variable
            double phiPoint; //!state variable

            double beta, Jz, rg, rv, alphag, alphav, alphaf, alphatheta, l,m, Jx; //parameters
            double a,psi;  //wind
            double eta;//viscosite
            double hv; // hauteur de centre de pousse

            int q;
            double cx,cy;// centre du triangle
            int iseg;// numero du segment

            void setTargetTriangle(double cx, double cy);
            void clock(double deltag,double deltavmax);
            void controller();
            explicit Sailboat(double x,double y, double dt);
        ~Sailboat();
};

#endif // sailboat_H
