#ifndef TORUS_H
#define TORUS_H

#include "hittable.h"
#include "solver.h"

class Torus : public Hittable {
public:
    Torus() {}

    Torus(double sweptR, double tubeR, shared_ptr<material> m) : sweptRadius(sweptR), tubeRadius(tubeR), mat_ptr(m),
        uScale(5.0), vScale(1.0)
    {}

    virtual bool hit(
        const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

    Box3 bbox() const override {
        // Taken from: https://www.iquilezles.org/www/articles/diskbbox/diskbbox.htm
        Point3 pa = - 0.5 * tubeRadius * Vec3(0, 1, 0);
        Point3 pb = pa + tubeRadius * Vec3(0, 1, 0);
        double ra = sweptRadius + tubeRadius;
        Vec3 a = pb - pa;
        double a_dot_a = a.dot(a);
        Vec3 axa = Vec3(a[0] * a[0], a[1] * a[1], a[2] * a[2]);
        Vec3 Q = Vec3(1.0, 1.0, 1.0) - axa / a_dot_a;
        Vec3 e = ra * Vec3(sqrt(Q[0]), sqrt(Q[1]), sqrt(Q[2]));
        Vec3 min, max;
        for (unsigned i = 0; i < 3; ++i) {
            min[i] = min(pa[i] - e[i], pb[i] - e[i]);
            max[i] = max(pa[i] + e[i], pb[i] + e[i]);
        }
        return Box3(min.cast<float>(), max.cast<float>());
    }

public:
    shared_ptr<material> mat_ptr;
    double sweptRadius;
    double tubeRadius;
    double uScale;
    double vScale;

private:
    Vec3 normalAtPoint(Point3 point) const;
    void get_torus_uv(const Point3& p, double& u, double& v) const;
};

bool Torus::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {

    // Taken from: http://blog.marcinchwedczuk.pl/ray-tracing-torus
    double ox = r.orig.x();
    double oy = r.orig.y();
    double oz = r.orig.z();

    double dx = r.dir.x();
    double dy = r.dir.y();
    double dz = r.dir.z();

    // define the coefficients of the quartic equation
    double sum_d_sqrd = dx * dx + dy * dy + dz * dz;
    double e = ox * ox + oy * oy + oz * oz - sweptRadius * sweptRadius - tubeRadius * tubeRadius;
    double f = ox * dx + oy * dy + oz * dz;
    double four_a_sqrd = 4.0 * sweptRadius * sweptRadius;

    double coeffs[5] = {
        e* e - four_a_sqrd * (tubeRadius * tubeRadius - oy * oy),
        4.0 * f * e + 2.0 * four_a_sqrd * oy * dy,
        2.0 * sum_d_sqrd * e + 4.0 * f * f + four_a_sqrd * dy * dy,
        4.0 * sum_d_sqrd * f,
        sum_d_sqrd* sum_d_sqrd
    };

    double sols[4];
    int solutions = solveQuartic(coeffs, sols);
    if (!solutions) return false;
    double root = infinity;
    for (unsigned i = 0; i < solutions; ++i) {
        if (sols[i] > 0.0001 && sols[i] < root) {
            root = sols[i];
        }
    }
    if (root < t_min || t_max < root)
        return false;

    rec.t = root;
    rec.p = r.at(rec.t);
    rec.normal = normalAtPoint(rec.p);
    get_torus_uv(rec.p, rec.u, rec.v);
    rec.mat_ptr = mat_ptr;

    return true;
}

Vec3 Torus::normalAtPoint(Point3 point) const
{
    double paramSquared = sweptRadius * sweptRadius + tubeRadius * tubeRadius;

    double x = point.x();
    double y = point.y();
    double z = point.z();
    double sumSquared = x * x + y * y + z * z;
    Vec3 normal = Vec3(4.0 * x * (sumSquared - paramSquared), 
        4.0 * y * (sumSquared - paramSquared + 2.0 * sweptRadius * sweptRadius),
        4.0 * z * (sumSquared - paramSquared));
    return normal.normalized();
}

void Torus::get_torus_uv(const Point3& p, double& u, double& v) const
{
    Vec3 j = Vec3(0, 1, 0);
    Vec3 D = p;
    Vec3 projDN = D.dot(j) * j;
    Vec3 R = D.normalized();
    Vec3 cR = (D - projDN).normalized();
    Vec3 C = cR * (sweptRadius + tubeRadius * 0.5);
    Vec3 V = (p - C).normalized();
    u = atan2(C.x(), C.z()) / pi;
    v = (acos(cR.dot(V)) / pi) * vScale;
    u = ((u + 1.0) * 0.5) * uScale;
    // repeat uv coord
    if (u > 1.0) {
        u -= int(u);
    }
    if (v > 1.0) {
        v -= int(v);
    }
}


#endif
