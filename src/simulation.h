#ifndef _simulation_h
#define _simulation_h

#include "Vec3.h"
#include "entities.h"

#include <iostream>
#include <fstream>
#include <cmath>

void runSimulation(
    Missile& m, 
    ArrivalPoint& target, 
    double dt, 
    double T, 
    double Thit, 
    const std::string& outputPath,
    double apex
    );

#endif