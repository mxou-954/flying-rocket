#include <iostream>
#include <fstream>
#include <cmath>

struct Vec3 {
    double x, y, z;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
};

struct Missile {
    Vec3 pos;
    Vec3 vel;
};

int main() {
    // Paramètres
    const double g = 9.81;      // m/s^2
    const double dt = 0.01;     // pas de temps (s)
    const double T  = 10.0;     // durée (s)

    // Missile initial
    Missile m;
    m.pos = {0.0, 0.0, 0.0};         // départ au sol
    m.vel = {80.0, 0.0, 60.0};       // vitesse initiale (m/s)

    // Accélération gravité (z = hauteur)
    Vec3 a = {0.0, 0.0, -g};

    std::ofstream out("traj.csv");
    out << "t,x,y,z\n";

    for (double t = 0.0; t <= T; t += dt) {
        // sauvegarde
        out << t << "," << m.pos.x << "," << m.pos.y << "," << m.pos.z << "\n";

        // intégration Euler
        m.vel += a * dt;
        m.pos += m.vel * dt;

        // stop quand on retouche le sol
        if (m.pos.z < 0.0) break;
    }

    std::cout << "Trajectoire écrite dans traj.csv\n";
    return 0;
}
