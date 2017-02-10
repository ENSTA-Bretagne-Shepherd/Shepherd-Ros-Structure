#ifndef Buoy_H
#define Buoy_H

#include <math.h>

#pragma once

class Buoy0
{
    public:
        double x;//!State variables
        double y;//!State variables
        double z;//!State variables        
        double Xdot[3];//!Vecteur de vitesse de Buoy
        
        int n;        
        
        inline Buoy0(int nb, double xb, double yb, double zb, double ub){
    		n = nb;
    		x = xb;
    		y = yb;
    		z = zb;    
		}

    inline Buoy0 (){x=0;y=0;z=-10;Xdot[0]=0;Xdot[1]=0;Xdot[2]=0;}
 };

/**
 * Classe Buoy : la bouee
 */
class Buoy : public Buoy0
{
public:
    double mvol;//!Masse volumique de Buoy
    double rho_w;//!Masse volumique de l'eau salée

    double beta;//!Parameter for Lorentz
    double sigma;//!Parameter for Lorentz
    double rho;//!Parameter for Lorentz
    double k;//!Parameter for Lorentz


    double volBal;//!Commande
    double Xdot2[3];//!Vecteur acceleration de Buoy
    double vx;//!composante du vecteur vitesse d'une particule dans un courant selon la direction x
    double vy;//!composante du vecteur vitesse d'une particule dans un courant selon la direction x
    double Dx;//!composante selon l'axe x du differentiel du vecteur vitesse par rapport au temps(Du/Dt)
    double Dy;//!composante selon l'axe y du differentiel du vecteur vitesse par rapport au temps(Du/Dt)
    double mu;//!coefficient de frottement
    double delta;//!delta = rapport de la masse volumique de l'eau sur celui de la bouee
    double theta;//!angle qui va permettre de rotater les champs de vecteurs sur le plan xy, pour chaque profondeur

    /**
     * Equations cinematiques pour les attracteurs de Lorentz
     */
    void lorenz(void);

    /**
     * Equations cinematiques simple pour une oscillation verticale
     */
    void sinLine(void);
    void pendulum(void);
    void stateEq(void);

    /**
     * Equations dynamique d'un vortex
     */
    void vortex(void);

    /**
     * Equation determinant la vitesse d'une particule dans le courant
     */
    void eqParticule(void);

    /**
     * Equation permettant de rotater les plan des courants
     */
    void rotation(void);

    /**
     * Setter for the command ub
     * @param ub
     */
    void setCommand(double ub);

    /**
     * Getter for the Id of the Buoy
     * @return id of the Buoy
     */
    int getNumber(void);
    double* getPos(void);
    void clock(void);

    /**
     * Constructeur de la bouee
     * @param nb : id de la bouee
     * @param xb : position x en m
     * @param yb : position y en m
     * @param zb : profondeur z en m
     * @param ub : commande
     */
    Buoy(int nb, double xb, double yb, double zb, double ub);

    inline Buoy (){x=0;y=0;z=-10;Xdot[0]=0;Xdot[1]=0;Xdot[2]=0;}
    //~Buoy();
};

#endif // Buoy_H
