#include <sailboat.h>
#include <stdio.h>

Sailboat::Sailboat(double x,double y,double dt)
{
    this->dt = dt;
    this->x=x; //m
    this->y=y; //m
    theta =-3.0; //deg
    v = 1; omega = 0.0;  phi = 0.5; phiPoint = 0;
    Jx = 3000.0; Jz = 10000.0; // moments d'inertie
    beta=0.1;  rg=2.0;  alphatheta=6000;  m=300.0;
    alphaf=1.0;  rv=1.0;  alphag=2000.0; l=1.0;  alphav=1000.0;
    a=2;
    psi=M_PI;  //vent
    eta = 16000;     hv = 4.00;  //Roulis
    //ax=-1000;ay=-2000,bx=1000;by=2000;   //sinLine with wind
    //ax=-1000;ay=0,bx=1000;by=0;   //sinLine against wind
    q=1;
    iseg=0;
}

Sailboat::~Sailboat()
{
}

void Sailboat::setTargetTriangle(double cx, double cy)
{
    this->cx = cx;
    this->cy = cy;
}

double sign(double a)
{if (a>0) return 1; else return -1;};

void Sailboat::clock(double deltag,double deltavmax)  // The model is described in "L. Jaulin Modelisation et commande d'un bateau a voile, CIFA2004, Douz (Tunisie)"
{
    double xw_ap=a*cos(psi-theta)-v; // cos s'utilise avec des radians
    double yw_ap=a*sin(psi-theta);
    double psi_ap=atan2(yw_ap,xw_ap);   //Apparent wind
    double a_ap=sqrt(xw_ap*xw_ap+yw_ap*yw_ap);
    gamma=cos(psi_ap)+cos(deltavmax);
    if (gamma<0) {deltav=M_PI+psi_ap;} //voile en drapeau
    else  if (sin(-psi_ap)>0) deltav=deltavmax;   else deltav=-deltavmax;
    fg = alphag*v*sin(deltag);
    fv = alphav*a_ap*sin(deltav-psi_ap);
    x += (v*cos(theta)+beta*a*cos(psi))*dt;
    y += (v*sin(theta)+beta*a*sin(psi))*dt;
    theta += omega*dt;
    omega += (1/Jz)*((l-rv*cos(deltav))*fv-rg*cos(deltag)*fg-alphatheta*omega*v)*dt;
    v     += (1/m)*(sin(deltav)*fv-sin(deltag)*fg-alphaf*v*v)*dt;
    phiPoint += (-phiPoint+fv*hv*cos(deltav)*cos(phi)/Jx - 10000*9.81*sin(phi)/Jx)*dt ;
    phi += phiPoint * dt;
    printf("Sailboat State : x : %f, y : %f, v : %f, theta : %f, omega : %f \n",x,y,v,theta,omega);
}

void Sailboat::setWindAccel(double a) {
    Sailboat::a = a;
}

void Sailboat::setWindDir(double psi) {
    Sailboat::psi = psi;
}
