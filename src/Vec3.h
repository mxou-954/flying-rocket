#ifndef _Vec3_h
#define _Vec3_h

#include <cmath>

struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}                        // constructeur vide
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}  // constructeur avec valeurs

    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
};
inline double length(const Vec3& v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }

#endif