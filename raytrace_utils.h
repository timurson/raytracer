#ifndef RAYTRACE_UTILS_H
#define RAYTRACE_UTILS_H

#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>

// Usings

using std::shared_ptr;
using std::make_shared;
using std::sqrt;

// Constants

const double infinity = std::numeric_limits<double>::infinity();
const float epsilon = std::numeric_limits<float>::epsilon();
const double pi = 3.1415926535897932385;

// Utility Functions

inline double degrees_to_radians(double degrees) {
    return degrees * pi / 180.0;
}

inline double radians_to_degrees(double radians) {
    return radians * 180.0 / pi;
}

inline double clamp(double x, double min, double max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

inline double random_double() {
    // Returns a random real in [0,1).
    //return rand() / (RAND_MAX + 1.0);

    static std::uniform_real_distribution<double> distribution(0.0, 1.0);
    static std::mt19937 generator;
    return distribution(generator);
}

inline double random_double(double min, double max) {
    // Returns a random real in [min,max).
    return min + (max - min) * random_double();
}

inline int random_int(int min, int max) {
    // Returns a random integer in [min,max].
    return static_cast<int>(random_double(min, max + 1));
}

inline Vec3 random_in_unit_sphere() {
    while (true) {
        auto p = Vec3::Random();
        if (p.squaredNorm() >= 1) continue;
        return p;
    }
}

inline Vec3 random_unit_vector() {
    return random_in_unit_sphere().normalized();
}

inline Vec3 random_in_hemisphere(const Vec3& normal) {
    Vec3 in_unit_sphere = random_in_unit_sphere();
    if (in_unit_sphere.dot(normal) > 0.0) // In the same hemisphere as the normal
        return in_unit_sphere;
    else
        return -in_unit_sphere;
}

inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2.0 * v.dot(n) * n;
}

inline bool near_zero(const Vec3& v) {
    // Return true if the vector is close to zero in all dimensions.
    const auto s = 1e-8;
    return (fabs(v.x()) < s) && (fabs(v.y()) < s) && (fabs(v.z()) < s);
}

inline bool approx_equal(double a, double b) {
    const auto s = 1e-8;
    return fabs(a - b) < s;
}

inline Vec3 randomVec3(double min, double max) {
    return Vec3(random_double(min, max), random_double(min, max), random_double(min, max));
}

/***************************************************************************/
// This function returns a squared distance between a given point and a
// line segment (start and end) (Adopted from Essential Mathematics book)
/**************************************************************************/
inline double distance_sqr(const Point3& start, const Point3& end, const Point3& point, double& t) {
    Vec3 w = point - start;
    Vec3 dir = end - start;
    auto proj = w.dot(dir);
    // start is closest point
    if (proj <= 0) {
        t = 0.0;
        return w.dot(w);
    }
    else {
        auto vsq = dir.dot(dir);
        // endpoint is closest point
        if (proj >= vsq) {
            t = 1.0;
            return w.dot(w) - 2.0 * proj + vsq;
        }
        // otherwise somewhere else in segment
        else {
            t = proj / vsq;
            return w.dot(w) - t * proj;
        }
    }
}

inline void closest_pt_point_segment(Point3 c, Point3 a, Point3 b, double& t, Point3& d)
{
    Vec3 ab = b - a;
    // Project c onto ab, but deferring divide by Dot(ab, ab)
    t = (c - a).dot(ab);
    if (t <= 0.0) {
        // c projects outsize the [a,b] interval, on the a size; clamp to a
        t = 0.0;
        d = a;
    }
    else {
        double denom = ab.dot(ab); // Always nonnegative since denom = ||ab||^2
        if (t >= denom) {
            // c projects outsize the [a,b] interval, on the b size; clamp to b
            t = 1.0;
            d = b;
        }
        else {
            // c projects inside the [a,b] interval; must do deferred divide now
            t = t / denom;
            d = a + t * ab;
        }
    }
}

#include "ray.h"



#endif
