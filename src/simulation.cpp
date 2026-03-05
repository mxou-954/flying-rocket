#include "simulation.h"

void runSimulation(
    Missile& m, 
    ArrivalPoint& target, 
    double dt, 
    double T, 
    double Thit, 
    const std::string& outputPath,
    double apex
) 
{
    const double g = 9.81;
    
    Vec3 gravity = Vec3(0, 0, -g);
    
    m.vel.z = std::sqrt(2 * g * apex);
    Thit = 2 * m.vel.z / g;
    m.vel.x = (target.pos.x - m.pos.x) / Thit;
    m.vel.y = (target.pos.y - m.pos.y) / Thit;
    std::cout << "vel initiale: " << m.vel.x << " " << m.vel.y << " " << m.vel.z << "\n";

    std::ofstream out(outputPath);
    out << "t,x,y,z,speed\n";

    double t = 0.0;
    for(; t <= T; t += dt) {
        double speed = length(m.vel);
        out << t << "," << m.pos.x << "," << m.pos.y << "," << m.pos.z
            << "," << speed << "\n";

        // La trainée c'est a résistance de l'air calculé 
        // k = m.drag
        // k est un coeficient de frottement
        
        // F = m * a ==> a = F / m
        // l'accélération causée par la trainée est :
        // F_drag = -k * vel
        // a_drag = F_drag / mass = -k * vel / mass
        
        Vec3 drag = m.vel * (-m.drag / m.mass); // m/s^2
        Vec3 a = gravity + drag; // m/s^2

        // Symplectic Euler integration
        m.vel += a  * dt;   // 1. update velocity with gravity
        m.pos += m.vel * dt; // 2. update position with new velocity

        if(m.pos.z <= 0){
            break;
        } 
    }

    std::cout << "Temps total de simulation " << t << " secondes.\n";
    std::cout << "Position finale du missile (x: " << m.pos.x <<", y: " << m.pos.y << ", z: " << m.pos.z << ")\n";
    std::cout << "Différence entre position finale de la Cible et du Missile (x: " << m.pos.x - target.pos.x <<", y: " << m.pos.y - target.pos.y << ", z: " << m.pos.z - target.pos.z << ")\n";
    std::cout << "Distance scalaire entre position finale de la Cible et du Missile: " << length(m.pos - target.pos) << "m" << std::endl;
}