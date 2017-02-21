#include <sailboat.h>
#include "buoy.h"
#include <stdio.h>
#include <stdlib.h>

Buoy::Buoy(int nb, double xb, double yb, double zb, double ub, double dt)
{

    this->dt = dt;

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
    //volBal = BUOY_CONTROL;
    volBal = 0;
    S = 1;

    // Caracteristiques physiques
    m = 100;
    vol = 100;
    rho_w=1.025;
    mvol = m/(vol-volBal); //kg/m³

    // Lorentz variables
    sigma = 10.0;
    beta = 8.0/3.0;
    rho = 28;
    k = 0.0;
    delta = mvol/rho_w; //delta = rapport de la masse volumique de l'eau sur celui de la bouee
    mu = 1.0; //coefficient de resistance de Stokes
    theta = 10;

    phi_i = 1.0;
    Ri = 100.0;

    int i =0;
    //TODO : tourne a linfini
    /*
    while (Xi[i]=!NULL)
    {
        double r = rand() % 200;
        double ang = rand() % 200;
        printf("rayon : %f angle : %f\n", r, ang);
        Xi[i] = r*cos(314*ang);
        Yi[i] = r*sin(314*ang);
        i++;
        printf("valeur de i :%i\n", i);
    }
    */
   printf("appel dans le constructeur :%f\n", Xi[0]);
    Xi[0] = 100;
    Xi[1] = 100;
    Xi[2] = -100;
    Xi[3] = -100;

    Yi[0] = 100;
    Yi[1] = -100;
    Yi[2] = -100;
    Yi[3] = 100;

}

void Buoy::lorenz(void)
{
    Xdot[0] = sigma*(y-x)*dt;
    Xdot[1] = (x*(rho-z)-y)*dt;
    Xdot[2] = (k*(x*y-beta*z)+u);
}

void Buoy::sinLine(double simuTime)
{
    double depth = 40; // m
    double freq = 0.05; // Hz
    double speed = 10;  // m/s
    Xdot[0] = 0;      //X
    Xdot[1] = 0;      //Y
    Xdot[2] = speed*sin(2*M_PI*simuTime*freq); //Z
}

void Buoy::pendulum(void)
{
    Xdot[0] = y;
    Xdot[1] = -sin(x);
    Xdot[2] = u;
}

void Buoy::stateEq(void)
{
    Xdot[0] = sin(0.001*(y+0.9*z));
    Xdot[1] = -sin(0.001*(x+z));
    Xdot[2] = u;
    
}

void Buoy::setCommand(double ub)
{
    u = ub;
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


void Buoy::eqParticuleSimple(void)
{
    vx = -2*sin(y);
    vy = 2*sin(x);

    Dx = -4*sin(x)*cos(y);
    Dy = 4*sin(y)*cos(x);
}

void Buoy::eqParticule(void)
{
    double d2;
    double dvxx, dvyx, dvxy, dvyy;
    vx = 0;
    vy = 0;
    dvxx = 0;
    dvyx = 0;
    dvxy = 0;
    dvyy = 0;

    for (int i=0;i<4;i++)
    {
        d2 = (x-Xi[i])*(x-Xi[i])+(y-Yi[i])*(y-Yi[i]);

        vx   += phi_i*2*y*(y-Yi[i])*exp(-1*d2/(Ri*Ri))/(Ri*Ri);
        vy   -= phi_i*2*x*(x-Xi[i])*exp(-1*d2/(Ri*Ri))/(Ri*Ri);

        dvxx -= phi_i*4*y*(y-Yi[i])*(x-Xi[i])*exp(-1*d2/(Ri*Ri))/(Ri*Ri*Ri*Ri);
        dvxy += 2*phi_i*exp(-1*d2/(Ri*Ri))/(Ri*Ri)*((2*y-Yi[i])-2*y*(y-Yi[i])*(y-Yi[i]));
        dvyx -= 2*phi_i*exp(-1*d2/(Ri*Ri))/(Ri*Ri)*((2*x-Xi[i])-2*x*(x-Xi[i])*(x-Xi[i]));
        dvyy += phi_i*4*x*(y-Yi[i])*(x-Xi[i])*exp(-1*d2/(Ri*Ri))/(Ri*Ri*Ri*Ri);
    }

    Dx = vx*dvxx+vy*dvxy;
    Dy = vx*dvyx+vy*dvyy;
}

void Buoy::vortex(void)
{
    //Dx = vy*0.5*(1+sin(theta*z))*2*cos(y);
    //Dy = vx*0.5*(1+sin(theta*z))*2*cos(x);
    eqParticule();
    mvol = m/(vol-volBal);
    delta = mvol/rho_w;
    Xdot2[0] = delta * Dx - mu * (Xdot[0] - vx);
    Xdot2[1] = Dy - mu * (Xdot[1] - vy);
    Xdot2[2] = (m - (vol - volBal) * rho_w) * 9.81 - mu * Xdot[2];
    printf("Buoy  mass : %f : vol : %f, volbal : %f, accel : %f \n",m,vol,volBal,Xdot2[2]);

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



void Buoy::clock(void)  // The model is described in "L. Jaulin Modélisation et commande d'un bateau à voile, CIFA2004, Douz (Tunisie)"
{
    // On met à jour la position de la bouee
    // On travaille en dynamique donc pfd m*a = Somme(Forces)
    vortex();
    rotation();
    Xdot[0] = Xdot[0]+dt*Xdot2[0];
    Xdot[1] = Xdot[1]+dt*Xdot2[1];
    Xdot[2] = Xdot[2]+dt*Xdot2[2];
    x = x+dt*Xdot[0];
    y = y+dt*Xdot[1];
    z = z+dt*Xdot[2];
    if (z<0)
    {
        z = 0;
        Xdot[2] = 0;
    }
    //TODO : mettre un min et un max
    volBal = volBal+u*S*dt;
    if(volBal<0){volBal = 0;}
    else if(volBal>50){volBal = 50;}
    printf("Buoy State %d : x : %f, y : %f, z : %f \n",n,x,y,z);
    fflush(stdout);
}

