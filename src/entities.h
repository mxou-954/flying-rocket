#ifndef _entities_h
#define _entities_h
#include "Vec3.h"

struct Missile { 
    Vec3 pos, vel; 
    Vec3   nose_dir;
    double mass = 0; 
    double fuel = 0; 
    double thrust = 0; 
    bool engineOn = false; 
    double burn_rate = 0;
    double max_angle_deg = 0;
    double cd = 0;
    double area = 0;
};

struct ArrivalPoint { 
    Vec3 pos; 
};

#endif