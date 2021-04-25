#ifndef HITTABLE_H
#define HITTABLE_H

#include "ray.h"


typedef Eigen::AlignedBox<float, 3> Box3; // The BV type provided by Eigen

class material;
class AABB;
class Hittable;

struct Slab
{
    Slab() {}
    Slab(Vector3f _normal, float _d0, float _d1) :normal(_normal), d0(_d0), d1(_d1) {}
    Vector3f normal;
    float d0, d1;
};

struct HitRecord {
    Point3 p;
    Vec3 normal;
    shared_ptr<material> mat_ptr;
    Hittable* obj_ptr;
    double t;
    double u;
    double v;
    bool front_face;
    inline void setFaceNormal(const Ray& r, const Vec3& outward_normal) {
        front_face = r.direction().dot(outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class Hittable {
public:
    Hittable() : area(0.0) {}
    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const = 0;
    virtual Box3 bbox() const { return Box3(); }
    double area;

};

class Translate : public Hittable {
public:
    Translate(shared_ptr<Hittable> p, const Vec3& displacement)
        : ptr(p), offset(displacement) {}
    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;
    virtual Box3 bbox() const override;

public:
    shared_ptr<Hittable> ptr;
    Vec3 offset;

};

bool Translate::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const
{
    Ray moved_r(r.origin() - offset, r.direction());
    if (!ptr->hit(moved_r, t_min, t_max, rec))
        return false;

    rec.p += offset;
    rec.setFaceNormal(moved_r, rec.normal);

    return true;
}

Box3 Translate::bbox() const
{
    Vector3f L = ptr->bbox().corner(Box3::BottomLeftFloor);
    Vector3f U = ptr->bbox().corner(Box3::TopRightCeil);
    return Box3(L + offset.cast<float>(), U + offset.cast<float>());
}

class RotateZ : public Hittable {
public:
    RotateZ(shared_ptr<Hittable> p, double angle);
    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;
    virtual Box3 bbox() const override {
        return bounding_box;
    };

public:
    shared_ptr<Hittable> ptr;
    double sin_theta;
    double cos_theta;
    Box3 bounding_box;
};

RotateZ::RotateZ(shared_ptr<Hittable> p, double angle) 
    : ptr(p)
{
    auto radians = degrees_to_radians(angle);
    sin_theta = sin(radians);
    cos_theta = cos(radians);
    Vector3f bmin = ptr->bbox().corner(Box3::BottomLeftFloor);
    Vector3f bmax = ptr->bbox().corner(Box3::TopRightCeil);

    Vector3f Min(infinity, infinity, infinity);
    Vector3f Max(-infinity, -infinity, -infinity);

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                auto x = i * bmax.x() + (1 - i) * bmin.x();
                auto y = j * bmax.y() + (1 - j) * bmin.y();
                auto z = k * bmax.z() + (1 - k) * bmin.z();

                auto newx = cos_theta * x + sin_theta * y;
                auto newy = -sin_theta * x + cos_theta * y;

                Vec3 tester(newx, newy, z);

                for (int c = 0; c < 3; c++) {
                    Min[c] = fmin(Min[c], tester[c]);
                    Max[c] = fmax(Max[c], tester[c]);
                }
            }
        }
    }

    bounding_box = Box3(Min, Max);
}

bool RotateZ::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const
{
    auto origin = r.origin();
    auto direction = r.direction();

    origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[1];
    origin[1] = sin_theta * r.origin()[0] + cos_theta * r.origin()[1];

    direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[1];
    direction[1] = sin_theta * r.direction()[0] + cos_theta * r.direction()[1];

    Ray rotated_r(origin, direction);
    if (!ptr->hit(rotated_r, t_min, t_max, rec))
        return false;

    auto p = rec.p;
    auto normal = rec.normal;

    p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[1];
    p[1] = -sin_theta * rec.p[0] + cos_theta * rec.p[1];

    normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[1];
    normal[1] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[1];

    rec.p = p;
    rec.setFaceNormal(rotated_r, normal);

    return true;

}

#endif
