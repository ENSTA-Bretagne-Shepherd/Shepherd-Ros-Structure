#include "buoy.h"
#include "config.h"
#include <stdio.h>
#include "sailboat.h"

extern double dt;
extern double simuTime;

Buoy::Buoy(int nb, double xb, double yb, double zb, double ub)
{
    // id
    n = nb;

    // Position
    x = xb;
    y = yb;
    z = zb;

    Xdot[0] = 0;
    Xdot[1] = 0;
    Xdot[2] = 0;

    Xdot2[0] = 0;
    Xdot2[1] = 0;
    Xdot2[2] = 0;

    // Command
    //volBal = ub;
    //volBal = BUOY_CONTROL;
    volBal = ub;

    // Caracteristiques physiques
    mvol = BUOY_MASS/(BUOY_VOLUME-volBal); //kg/m³

    // Lorentz variables
    sigma = 10.0;
    beta = 8.0/3.0;
    rho = 28;
    k = 0.0;
    rho_w = RHO_SALT_WATER;
    delta = mvol/rho_w; //delta = rapport de la masse volumique de l'eau sur celui de la bouee
    mu = 1.0; //coefficient de resistance de Stokes
    theta = 10;
}

void Buoy::lorenz(void)
{
    Xdot[0] = sigma*(y-x)*dt;
    Xdot[1] = (x*(rho-z)-y)*dt;
    Xdot[2] = (k*(x*y-beta*z));//attention au terme de commande
}

void Buoy::sinLine(void)
{
    double depth = 40; // m
    double freq = 0.05; // Hz
    double speed = 10;  // m/s
    Xdot[0] = 0;      //X
    Xdot[1] = 0;      //Y
    //TODO : prendre en entree simutime
    //Xdot[2] = speed*sin(2*M_PI*simuTime*freq); //Z
}

void Buoy::pendulum(void)
{
    Xdot[0] = y;
    Xdot[1] = -sin(x);
    Xdot[2] = volBal;
}

void Buoy::stateEq(void)
{
    Xdot[0] = sin(0.001*(y+0.9*z));
    Xdot[1] = -sin(0.001*(x+z));
    Xdot[2] = volBal;

}

void Buoy::eqParticule(void)
{
    vx = -2*sin(y);
    vy = 2*sin(x);
}

void Buoy::vortex(void)
{
    //Dx = vy*0.5*(1+sin(theta*z))*2*cos(y);
    //Dy = vx*0.5*(1+sin(theta*z))*2*cos(x);
    eqParticule();
    Dx = -4*sin(x)*cos(y);
    Dy = 4*sin(y)*cos(x);
    mvol = BUOY_MASS/(BUOY_VOLUME-volBal);
    delta = mvol/RHO_SALT_WATER;
    Xdot2[0] = delta * Dx - mu * (Xdot[0] - vx);
    Xdot2[1] = Dy - mu * (Xdot[1] - vy);
    Xdot2[2] = (BUOY_MASS-(BUOY_VOLUME-volBal)*rho_w)*GRAV_CONST;
}

void Buoy::rotation(void)
{
    double xd;
    double yd;
    xd = cos(theta*z)*vx-sin(theta*z)*vy;
    yd = cos(theta*z)*vy+sin(theta*z)*vx;
    vx = xd;
    vy = yd;
}

void Buoy::setCommand(double ub)
{
    volBal = ub;
}

int Buoy::getNumber(void)
{
    return n;
}

double* Buoy::getPos(void)
{
    double* xd = new double[4];
    xd[0] = sqrt(pow(Xdot[0],2.0)+pow(Xdot[1],2.0)+pow(Xdot[2],2.0));
    xd[1] = x;
    xd[2] = y;
    xd[3] = z;
    return xd;
}

void Buoy::clock(void)  // The model is described in "L. Jaulin Modélisation et commande d'un bateau à voile, CIFA2004, Douz (Tunisie)"
{

    // On met à jour la position de la bouee
    // On travaille en dynamique donc pfd m*a = Somme(Forces)
    //lorenz();
/*
    printf("Buoy acc x : %f speed : %f \n",Xdot2[0],Xdot[0]);
    printf("Buoy acc y : %f speed : %f \n",Xdot2[1],Xdot[1]);
    printf("Buoy acc z : %f speed : %f \n\n\n",Xdot2[2],Xdot[2]);
*/
    vortex();
    rotation();
/*
    printf("Buoy acc x : %f speed : %f \n",Xdot2[0],Xdot[0]);
    printf("Buoy acc y : %f speed : %f \n",Xdot2[1],Xdot[1]);
    printf("Buoy acc z : %f speed : %f \n",Xdot2[2],Xdot[2]);
    */
    Xdot[0] = Xdot[0]+dt*Xdot2[0];
    Xdot[1] = Xdot[1]+dt*Xdot2[1];
    Xdot[2] = Xdot[2]+dt*Xdot2[2];
    x = x+dt*Xdot[0];
    y = y+dt*Xdot[1];
    z = z+dt*Xdot[2];

    //printf("Buoy acc : %f speed : %f \n",Xdot2[0],Xdot[0]);
    printf("Buoy state %d : %f %f %f \n",n,x,y,z);
    fflush(stdout);
}
