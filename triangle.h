#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"

class Triangle : public Hittable {
public:
    Triangle() {}
    Triangle(Point3 p0, Point3 p1, Point3 p2, shared_ptr<material> m)
        : a(p0), b(p1), c(p2), mat_ptr(m) {
        normal = (b - a).cross((c - a));
    }
    Triangle(unsigned int i0, unsigned int i1, unsigned int i2, shared_ptr<material> m, MeshData* mesh);

    virtual bool hit(
        const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

    Box3 bbox() const override {
        Vector3f min, max;
        for (int i = 0; i < 3; ++i) {
            if (a[i] > b[i])
                if (b[i] > c[i]) min[i] = c[i];
                else min[i] = b[i];
            else if (a[i] > c[i])min[i] = c[i];
            else min[i] = a[i];

            if (a[i] < b[i])
                if (b[i] < c[i]) max[i] = c[i];
                else max[i] = b[i];
            else if (a[i] < c[i])max[i] = c[i];
            else max[i] = a[i];
        }
        return Box3(min, max);
    }

public:
    Point3 a;
    Point3 b;
    Point3 c;
    Vec3 a_n;
    Vec3 b_n;
    Vec3 c_n;
    Vec3 normal; // face normal for intersection test
    unsigned int index0;
    unsigned int index1;
    unsigned int index2;
    shared_ptr<material> mat_ptr;
    MeshData* mesh_ptr;

};

Triangle::Triangle(unsigned int i0, unsigned int i1, unsigned int i2, shared_ptr<material> m, MeshData* mesh)
    : index0(i0), index1(i1), index2(i2), mat_ptr(m), mesh_ptr(mesh) {

    a = mesh->vertices.at(index0).pnt.cast<double>();
    b = mesh->vertices.at(index1).pnt.cast<double>();
    c = mesh->vertices.at(index2).pnt.cast<double>();
    // retrieve normals too
    a_n = mesh->vertices.at(index0).nrm.cast<double>();
    b_n = mesh->vertices.at(index1).nrm.cast<double>();
    c_n = mesh->vertices.at(index2).nrm.cast<double>();
    normal = (b - a).cross(c - a);
}

bool Triangle::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
    Vec3 e1 = b - a;
    Vec3 e2 = c - a;
    Vec3 p = r.direction().cross(e2);

    double d = p.dot(e1);
    if (abs(d) < epsilon) return false;
    Vec3 s = r.origin() - a;
    double u = (p.dot(s)) / d;
    if (u < 0.0 || u > 1) return false;
    Vec3 q = s.cross(e1);
    double v = r.direction().dot(q) / d;
    if (v < 0.0 || u + v > 1.0) return false;
    double t = e2.dot(q) / d;
    if (t < t_min || t > t_max) return false;
 
    rec.t = t;
    rec.p = r.at(t);
    Vec3 outward_normal = (1.0 - u - v) * a_n + u * b_n + v * c_n;
    //Vec3 outward_normal = normal;
    rec.setFaceNormal(r, outward_normal.normalized());
    rec.mat_ptr = mat_ptr;

    return true;

}



#endif
