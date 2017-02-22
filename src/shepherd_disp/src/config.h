//
// Created by tag on 16/12/16.
//

#pragma once

// Interface
#define LARGEUR_FENETRE 900
#define HAUTEUR_FENETRE 700
#define DELAY_FRAME 30

// World
#define DT 0.05 // s
#define GRAV_CONST 9.81 // m/s²
#define RHO_SALT_WATER 1.025 // kg/m³
#define BOAT_NUMBER 4
#define BUOY_NUMBER 5
#define X_BOAT_TARGET {100,-100, 100,-100}
#define Y_BOAT_TARGET {100, 100,-100,-100}

// Buoy
#define BUOY_MASS 10.25 // kg
#define BUOY_VOLUME 10 // m³
#define BUOY_MAX_BAL_VOL 5.0 // m³

// Communication
#define ADRESS "127.0.0.1"
#define PORT "3000"
