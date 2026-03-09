// roquette M31 du LRU 

#include "simulation.h"

double airDensity(double altitude) {
    const double rho0 = 1.225;
    const double H = 8500.0;
    return rho0 * std::exp(-altitude / H);
}

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

    std::ofstream out(outputPath);
    out << "t,x,y,z,speed,x_theory,y_theory,z_theory,a_thrust_x,a_thrust_y,a_thrust_z\n";

    double t = 0.0;
    double engineSetOff = 0.0;
    Vec3 positionEngineOff = Vec3(0, 0, 0);
    m.engineOn = true;
    double initial_dist = length(target.pos - m.pos);
    Vec3 start_pos = m.pos;
    Vec3 wind = Vec3(0, 0, 0);

    double vx0 = (target.pos.x - start_pos.x) / Thit;
    double vy0 = (target.pos.y - start_pos.y) / Thit;
    double vz0 = apex * 4.0 / Thit; // dérivée de z_theory à t=0
    m.vel = Vec3(vx0, vy0, vz0);

    for(; t <= T; t += dt) {

        double wind_speed = ((double)rand() / RAND_MAX) * 15.0; // 0-15 m/s
        double wind_angle = ((double)rand() / RAND_MAX) * 2 * M_PI; // direction horizontale
        Vec3 wind = Vec3(
            wind_speed * cos(wind_angle),
            wind_speed * sin(wind_angle),
            0.0  // pas de vent vertical
        );

        if((m.fuel <= 0.5) && m.engineOn) {
            m.engineOn = false;
            engineSetOff = t;
            positionEngineOff = m.pos;
        }

        double z_theory = apex * 4.0 * (t / Thit) * (1.0 - t / Thit);
        double ratio = std::min(t / Thit, 1.0);
        Vec3 pos_theory = Vec3(
            start_pos.x + (target.pos.x - start_pos.x) * ratio,
            start_pos.y + (target.pos.y - start_pos.y) * ratio,
            z_theory
        );

        Vec3 a_thrust = Vec3(0, 0, 0);
        if(m.engineOn) {
            double z_error = pos_theory.z - m.pos.z;  // écart vertical uniquement
            double thrust_factor = std::min(std::max(z_error / 500.0, 0.0), 1.0);
            a_thrust = Vec3(0, 0, m.thrust * thrust_factor / m.mass);
            m.fuel -= m.burn_rate * thrust_factor * dt;
            m.mass -= m.burn_rate * thrust_factor * dt;
        }
        
        double speed = length(m.vel);
        out << t << "," << m.pos.x << "," << m.pos.y << "," << m.pos.z
            << "," << speed << "," << pos_theory.x << "," << pos_theory.y << "," << pos_theory.z << "," 
            << a_thrust.x << "," << a_thrust.y << "," << a_thrust.z << "\n";

        // La trainée c'est a résistance de l'air calculé 
        // k = m.drag
        // k est un coeficient de frottement
        
        // F = m * a ==> a = F / m
        // l'accélération causée par la trainée est :
        // F_drag = -k * vel
        // a_drag = F_drag / mass = -k * vel / mass
        
        double rho = airDensity(m.pos.z);
        double speed_relative = length(m.vel - wind);       // vitesse relative à l'air
        Vec3 vel_relative_dir = (m.vel - wind) * (1.0 / speed_relative);
        Vec3 drag = vel_relative_dir * (-0.5 * rho * m.cd * m.area * speed_relative * speed_relative / m.mass);
        Vec3 a = gravity + drag + a_thrust; // m/s^2

        double dz_theory = apex * 4.0 / Thit * (1.0 - 2.0 * t / Thit); // dérivée de z_theory
        double dx_theory = (target.pos.x - start_pos.x) / Thit;
        double dy_theory = (target.pos.y - start_pos.y) / Thit;
        Vec3 vel_theory = Vec3(dx_theory, dy_theory, dz_theory);

        Vec3 pos_error = pos_theory - m.pos;
        Vec3 vel_error = vel_theory - m.vel;
        Vec3 correction = pos_error * 1.0 + vel_error * 1.2;
        double correction_len = length(correction);

        if(correction_len > 0) {
            Vec3 correction_dir = correction * (1.0 / correction_len);
            double max_angle_rad = m.max_angle_deg * M_PI / 180.0;
            double max_correction = length(m.vel) * std::tan(max_angle_rad);
            double correction_magnitude = std::min(correction_len, max_correction);
            m.vel += correction_dir * correction_magnitude * dt;
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