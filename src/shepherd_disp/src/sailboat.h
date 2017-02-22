#ifndef sailboat_H
#define sailboat_H

#include <math.h>

#include <GL/gl.h>
#include <GL/glu.h>

#pragma once


class Sailboat0
{	
	public:
	
            double x,y,theta,phi;
            
            double fg,fv,gamma,deltav,deltag,deltavmax;  //link variables
	 		double a,psi;  //wind
            
            
            double cx,cy;// centre du triangle
};

/**
 * Classe bu bateau
 */
class Sailboat : public Sailboat0
{    public:
            // Variables d'etat
            int n; //!state variable
            
            double v; //!state variable
            double omega; //!state variable
            double phiPoint; //!state variable

            double beta, Jz, rg, rv, alphag, alphav, alphaf, alphatheta, l,m, Jx; //parameters
            double eta;//viscosite
            double hv; // hauteur de centre de pousse

            int q;
            int iseg;// numero du segment

            void setTargetTriangle(double cx, double cy);
            void clock();
            void controller();
            explicit Sailboat(int n,double x,double y);
        ~Sailboat();
};

#endif // sailboat_H
