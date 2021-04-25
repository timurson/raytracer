#ifndef CYLINDER_H
#define CYLINDER_H

#include "aarect.h"
#include "hittable_list.h"

class Cylinder : public Hittable {
public:
    Cylinder() {}
   /* Cylinder(const Point3& p0, const Point3& p1, double r, shared_ptr<material> ptr)
        : a(p0), b(p1), radius(r), mat_ptr(ptr) 
    {
        dir = (a - b).normalized();
    }*/

    Cylinder(Point3 b, Vec3 a, double r, shared_ptr<material> ptr) :
        base(b), axis(a), radius(r), mat_ptr(ptr),
        slab(Vector3f(0.0f, 0.0f, 1.0f), 0.0f, -a.norm()),
        q(Quaternionf::FromTwoVectors(a.cast<float>(), Vector3f::UnitZ()))
    {

    }

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

    Box3 bbox() const override {
        Vec3 min = base, max;
        for (int i = 0; i < 3; ++i)
            if (axis[i] > 0) max[i] = base[i] + axis[i];
            else {
                max[i] = base[i];
                min[i] = base[i] + axis[i];
            }
        return Box3(min.cast<float>() - Vector3f(radius, radius, radius), max.cast<float>() + Vector3f(radius, radius, radius));
    }

public:
    Quaternionf q;
    Slab slab;

    Point3 base;
    Vec3 axis;

    double radius;
    Vec3 dir;
    shared_ptr<material> mat_ptr;
};


bool Cylinder::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {

    Vector3f Q = q._transformVector(r.orig.cast<float>() - base.cast<float>());
    Vector3f D = q._transformVector(r.dir.cast<float>());
    float t0 = 0, t1 = FLT_MAX;
    double b0, b1;
    Vector3f normalVec;
    if (slab.normal.dot(D)) {
        double NdotQ = slab.normal.dot(Q);
        double NdotD = slab.normal.dot(D);
        b0 = -(slab.d0 + NdotQ) / NdotD;
        b1 = -(slab.d1 + NdotQ) / NdotD;
        if (b0 > b1) {
            double temp = b0;
            b0 = b1;
            b1 = temp;
        }
    }
    else {
        float NdotQ = slab.normal.dot(Q);
        float s0 = NdotQ + slab.d0;
        float s1 = NdotQ + slab.d1;
        if (s0 * s1 < 0.0f) {
            b0 = 0.0f;
            b1 = FLT_MAX;
        }
        else return false;
    }
    double a = D[0] * D[0] + D[1] * D[1];
    double b = 2 * (D[0] * Q[0] + D[1] * Q[1]);
    double c = Q[0] * Q[0] + Q[1] * Q[1] - radius * radius;
    double deter = b * b - 4.0 * a * c;
    if (deter < 0) return false;
    double c0 = (-b - sqrt(deter)) / 2.0f / a;
    double c1 = (-b + sqrt(deter)) / 2.0f / a;

    Vector3f normal0, normal1, M;
    if (b0 > c0) {
        t0 = b0;
        if (D[2] > epsilon) normal0 = Vector3f(0.0f, 0.0f, -1.0f);
        else normal0 = Vector3f(0.0f, 0.0f, 1.0f);
    }
    else {
        t0 = c0;
        M = Q + t0 * D;
        normal0 = Vector3f(M[0], M[1], 0.0f);
    }

    if (b1 < c1) {
        t1 = b1;
        if (D[2] > epsilon) normal1 = Vector3f(0.0f, 0.0f, 1.0f);
        else normal1 = Vector3f(0.0f, 0.0f, -1.0f);
    }
    else {
        t1 = c1;
        M = Q + t1 * D;
        normal1 = Vector3f(M[0], M[1], 0.0f);
    }
    if (t0 > t1) return false;
    else if (t0 > epsilon) {
        rec.t = t0;
        rec.normal = q.conjugate()._transformVector(normal0).cast<double>();
    }
    else if (t1 > epsilon) {
        rec.t = t1;
        rec.normal = q.conjugate()._transformVector(normal1).cast<double>();
    }
    else return false;

    // depth test check
    if (rec.t < t_min || t_max < rec.t)
        return false;

    rec.p = r.at(rec.t);
    rec.normal.normalize();
    rec.mat_ptr = mat_ptr;


    return true;
}

#endif