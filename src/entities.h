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
};

struct ArrivalPoint { 
    Vec3 pos; 
};

#endif