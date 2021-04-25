#ifndef AABB_H
#define AABB_H

#include "raytrace_utils.h"

class AABB {
public:
    AABB() {}
    AABB(const Point3& a, const Point3& b) { minimum = a; maximum = b; }

    Point3 min_pt() const { return minimum; }
    Point3 max_pt() const { return maximum; }

    bool hit(const Ray& r, double t_min, double t_max) const {
        for (int a = 0; a < 3; a++) {
            auto t0 = fmin((minimum[a] - r.origin()[a]) / r.direction()[a],
                (maximum[a] - r.origin()[a]) / r.direction()[a]);
            auto t1 = fmax((minimum[a] - r.origin()[a]) / r.direction()[a],
                (maximum[a] - r.origin()[a]) / r.direction()[a]);
            t_min = fmax(t0, t_min);
            t_max = fmin(t1, t_max);
            if (t_max <= t_min)
                return false;
        }
        return true;
    }

    double area() const {
        auto a = maximum.x() - minimum.x();
        auto b = maximum.y() - minimum.y();
        auto c = maximum.z() - minimum.z();
        return 2 * (a * b + b * c + c * a);
    }

public:
    Point3 minimum;
    Point3 maximum;

};

#endif
