#include <iostream>
#include <fstream>
#include <cmath>

struct Vec3 {
    double x, y, z;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s};   }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
};
double length(const Vec3& v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }

struct Missile      { Vec3 pos, vel; };
struct ArrivalPoint { Vec3 pos; };

int main() {
    const double g    = 9.81;
    const double dt   = 0.01;
    const double T    = 10.0;   // max simulation time (s)
    const double Thit = 5.0;    // desired time-of-flight to target (s)

    ArrivalPoint arvPT;
    arvPT.pos = {300, 25, 100};

    Missile m;
    m.pos = {1, 1, 1};

    const Vec3 a = {0, 0, -g};  // constant gravity acceleration

    // Ballistic initial velocity: pos(Thit) = p0 + v0*Thit + 0.5*a*Thit²
    // => v0 = (target - p0 - 0.5*a*Thit²) / Thit
    m.vel = (arvPT.pos - m.pos - a * (0.5 * Thit * Thit)) * (1.0 / Thit);

    std::ofstream out("traj.csv");
    out << "t,x,y,z,speed\n";

    for (double t = 0.0; t <= T; t += dt) {
        double speed = length(m.vel);
        out << t << "," << m.pos.x << "," << m.pos.y << "," << m.pos.z
            << "," << speed << "\n";

        // Symplectic Euler integration
        m.vel += a  * dt;   // 1. update velocity with gravity
        m.pos += m.vel * dt; // 2. update position with new velocity
    }

    std::cout << "OK -> traj.csv (Thit=" << Thit << "s)\n";
}