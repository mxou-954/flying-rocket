#ifndef _entities_h
#define _entities_h
#include "Vec3.h"

struct Missile { 
    Vec3 pos, vel; 
    double mass; 
    double drag; 
    double fuel; 
    double thrust; 
    bool engineOn; 
    double burn_rate;
    double guidance;
};

struct ArrivalPoint { 
    Vec3 pos; 
};

#endif