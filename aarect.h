#ifndef AARECT_H
#define AARECT_H

#include "hittable.h"

// represents an XY plane aligned rectangle
class XY_Rect : public Hittable {
public:
    XY_Rect() {}

    XY_Rect(
        double _x0, double _x1, double _y0, double _y1, double _k, shared_ptr<material> mat
    ) : x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k), mp(mat) {};

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

public:
    shared_ptr<material> mp;
    double x0, x1, y0, y1, k;
};

// represents an XY plane aligned rectangle
class XZ_Rect : public Hittable {
public:
    XZ_Rect() {}

    XZ_Rect(
        double _x0, double _x1, double _z0, double _z1, double _k, shared_ptr<material> mat
    ) : x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

public:
    shared_ptr<material> mp;
    double x0, x1, z0, z1, k;
};

// represents an YZ plane aligned rectangle
class YZ_Rect : public Hittable {
public:
    YZ_Rect() {}

    YZ_Rect(
        double _y0, double _y1, double _z0, double _z1, double _k, shared_ptr<material> mat
    ) : y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

public:
    shared_ptr<material> mp;
    double y0, y1, z0, z1, k;
};

bool XY_Rect::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
    auto t = (k - r.origin().z()) / r.direction().z();
    if (t < t_min || t > t_max)
        return false;

    auto x = r.origin().x() + t * r.direction().x();
    auto y = r.origin().y() + t * r.direction().y();
    if (x < x0 || x > x1 || y < y0 || y > y1)
        return false;

    rec.u = (x - x0) / (x1 - x0);
    rec.v = (y - y0) / (y1 - y0);
    rec.t = t;
    auto outward_normal = Vec3(0, 0, 1);
    rec.setFaceNormal(r, outward_normal);
    rec.mat_ptr = mp;
    rec.p = r.at(t);

    return true;
}

bool XZ_Rect::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
    auto t = (k - r.origin().y()) / r.direction().y();
    if (t < t_min || t > t_max)
        return false;

    auto x = r.origin().x() + t * r.direction().x();
    auto z = r.origin().z() + t * r.direction().z();
    if (x < x0 || x > x1 || z < z0 || z > z1)
        return false;

    rec.u = (x - x0) / (x1 - x0);
    rec.v = (z - z0) / (z1 - z0);
    rec.t = t;
    auto outward_normal = Vec3(0, 1, 0);
    rec.setFaceNormal(r, outward_normal);
    rec.mat_ptr = mp;
    rec.p = r.at(t);

    return true;
}

bool YZ_Rect::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
    auto t = (k - r.origin().x()) / r.direction().x();
    if (t < t_min || t > t_max)
        return false;

    auto y = r.origin().y() + t * r.direction().y();
    auto z = r.origin().z() + t * r.direction().z();
    if (y < y0 || y > y1 || z < z0 || z > z1)
        return false;

    rec.u = (y - y0) / (y1 - y0);
    rec.v = (z - z0) / (z1 - z0);
    rec.t = t;
    auto outward_normal = Vec3(1, 0, 0);
    rec.setFaceNormal(r, outward_normal);
    rec.mat_ptr = mp;
    rec.p = r.at(t);

    return true;
}


#endif
