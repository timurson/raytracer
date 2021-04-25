#ifndef RAY_H
#define RAY_H

#include <Eigen/StdVector> // For vectors, matrices (2d,3d,4d) and quaternions in f and d precision.

using Point3 = Vector3d;
using Vec3 = Vector3d;

using namespace Eigen;

class Ray {
public:
    Ray() {}
    Ray(const Point3& origin, const Vec3& direction)
        : orig(origin), dir(direction), tm(0)
    {}

    Ray(const Point3& origin, const Vec3& direction, double time)
        : orig(origin), dir(direction), tm(time)
    {}

    Point3 origin() const { return orig; }
    Vec3 direction() const { return dir; }
    double time() const { return tm; }

    Point3 at(double t) const {
        return orig + t * dir;
    }

public:
    Point3 orig;
    Vec3 dir;
    double tm;

};


#endif
