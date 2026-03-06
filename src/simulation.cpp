// roquette M31 du LRU 

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
    
    Thit = 2.0 * std::sqrt(2.0 * apex / g);
    std::cout << "Thit " << Thit << " secondes.\n";

    m.vel = Vec3(0, 0, 3);

    std::ofstream out(outputPath);
    out << "t,x,y,z,speed\n";

    double t = 0.0;
    double engineSetOff = 0.0;
    Vec3 positionEngineOff = Vec3(0, 0, 0);
    m.engineOn = true;
    double initial_dist = length(target.pos - m.pos);
    Vec3 start_pos = m.pos;

    for(; t <= T; t += dt) {

        if((m.fuel <= 0) && m.engineOn) {
            m.engineOn = false;
            engineSetOff = t;
            positionEngineOff = m.pos;
        }

        double z_theory = apex * 4.0 * (t / Thit) * (1.0 - t / Thit);
        double ratio = t / Thit;
        Vec3 pos_theory = Vec3(
            start_pos.x + (target.pos.x - start_pos.x) * ratio,
            start_pos.y + (target.pos.y - start_pos.y) * ratio,
            z_theory
        );

        double speed = length(m.vel);
        out << t << "," << m.pos.x << "," << m.pos.y << "," << m.pos.z
            << "," << speed << "\n";

        Vec3 a_thrust = Vec3(0, 0, 0);
        if(m.engineOn) {
            a_thrust = Vec3(0, 0, m.thrust / m.mass);
            m.fuel -= m.burn_rate * dt;
            m.mass -= m.burn_rate * dt;
        }

        // La trainée c'est a résistance de l'air calculé 
        // k = m.drag
        // k est un coeficient de frottement
        
        // F = m * a ==> a = F / m
        // l'accélération causée par la trainée est :
        // F_drag = -k * vel
        // a_drag = F_drag / mass = -k * vel / mass
        
        Vec3 drag = m.vel * (-m.drag / m.mass); // m/s^2
        Vec3 a = gravity + drag + a_thrust; // m/s^2

        if(t <= Thit) {
            // Phase montée : corriger vers la position théorique sur la parabole
            Vec3 correction = pos_theory - m.pos;
            m.vel += correction * m.guidance * dt;
        } else {
            // Phase descente : pointer vers la cible
            Vec3 to_target = target.pos - m.pos;
            double dist = length(to_target);
            Vec3 guidance_dir = to_target * (1.0 / dist);
            double guidance_factor = m.guidance * (dist / initial_dist);
            m.vel += guidance_dir * guidance_factor * dt;
        }

        // Symplectic Euler integration
        m.vel += a  * dt;   // physique : gravité + drag + poussée
        m.pos += m.vel * dt; // 2. update position with new velocity

        if(m.pos.z <= 0){
            break;
        } 
    }

    std::cout << "Temps total de simulation " << t << " secondes.\n";
    std::cout << "Position finale du missile (x: " << m.pos.x <<", y: " << m.pos.y << ", z: " << m.pos.z << ")\n";
    std::cout << "Différence entre position finale de la Cible et du Missile (x: " << m.pos.x - target.pos.x <<", y: " << m.pos.y - target.pos.y << ", z: " << m.pos.z - target.pos.z << ")\n";
    std::cout << "Distance scalaire entre position finale de la Cible et du Missile: " << length(m.pos - target.pos) << "m" << std::endl;
    std::cout << "Le moteur s'est arrété au bout de : " << engineSetOff << std::endl;
    std::cout << "Le moteur s'est arrêté aux coordonnées : (" 
          << positionEngineOff.x << ", " 
          << positionEngineOff.y << ", " 
          << positionEngineOff.z << ")\n";
    std::cout << "Le moteur a dépensé : " << (m.burn_rate * engineSetOff) << " kg de fuel\n";  
}