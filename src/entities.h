#ifndef _entities_h
#define _entities_h
#include "Vec3.h"

struct Missile { 
    Vec3 pos, vel; 
    double mass; 
    double fuel; 
    double thrust; 
    bool engineOn; 
    double burn_rate;
    double max_angle_deg;
    double cd;
    double area;
};

struct ArrivalPoint { 
    Vec3 pos; 
};

#endif