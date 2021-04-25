#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"

class Sphere : public Hittable {
public:
    Sphere() {}

    Sphere(Point3 cen, double r, shared_ptr<material> m)
        : center(cen), radius(r), mat_ptr(m) 
    {
        Hittable::area = 4.0 * pi * radius * radius;
    }

    virtual bool hit(
        const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

    Box3 bbox() const override {
        return Box3(Vector3f(center[0] - radius, center[1] - radius, center[2] - radius),
            Vector3f(center[0] + radius, center[1] + radius, center[2] + radius));
    }

public:
    Point3 center;
    double radius;
    shared_ptr<material> mat_ptr;

private:
    static void get_sphere_uv(const Point3& p, double& u, double& v) {
        // p: a given point on the sphere of radius one, centered at the origin.
        // u: returned value [0,1] of angle around the Z axis from X=-1.
        // v: returned value [0,1] of angle from Z=-1 to Z=+1.
        //     <1 0 0> yields <0.50 0.50>       <-1  0  0> yields <0.00 0.50>
        //     <0 0 1> yields <0.50 1.00>       < 0  0 -1> yields <0.50 0.00>
        //     <0 1 0> yields <0.25 0.50>       < 0  -1 0> yields <0.75 0.50>
        auto theta = acos(-p.z());
        auto phi = atan2(p.y(), p.x()) + pi;
        u = phi / (2 * pi);
        v = theta / pi;
    }
};

bool Sphere::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {

    Vec3 oc = r.origin() - center;
    auto a = r.direction().squaredNorm();
    auto half_b = oc.dot(r.direction());
    auto c = oc.squaredNorm() - radius * radius;

    auto discriminant = half_b * half_b - a * c;
    if (discriminant < 0) return false;
    auto sqrtd = sqrt(discriminant);

    // Find the nearest root that lies in the acceptable range.
    auto root = (-half_b - sqrtd) / a;
    if (root < t_min || t_max < root) {
        root = (-half_b + sqrtd) / a;
        if (root < t_min || t_max < root)
            return false;
    }

    rec.t = root;
    rec.p = r.at(rec.t);
    Vec3 outward_normal = (rec.p - center) / radius;
    //rec.normal = outward_normal; // for refractive spheres
    rec.setFaceNormal(r, outward_normal);
    get_sphere_uv(outward_normal, rec.u, rec.v);
    rec.mat_ptr = mat_ptr;

    return true;
}


#endif
