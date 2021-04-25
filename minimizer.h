#ifndef MINIMIZER_H
#define MINIMIZER_H

#include "raytrace_utils.h"
#include "hittable.h"
#include "ray.h"


class Minimizer {
public:
    typedef float Scalar;  // KdBVH needs Minimizer::Scalar defined
    Ray ray;
    HitRecord minInt; // minimum intersection record

    // Constructor
    Minimizer(const Ray& r) : ray(r) { minInt.t = infinity; }
    Minimizer() {}

    Scalar minimumOnObject(Hittable* obj);
    Scalar minimumOnVolume(const Box3& box);

};

Minimizer::Scalar Minimizer::minimumOnObject(Hittable* obj) {
    HitRecord rec;
    if (obj->hit(ray, 0.0001, FLT_MAX, rec)) {
        if (rec.t < minInt.t) {
			rec.obj_ptr = obj;
            minInt = rec;
            return rec.t;
        }
    }

    return FLT_MAX;
}

Minimizer::Scalar Minimizer::minimumOnVolume(const Box3& box) {
	Vector3f L = box.corner(Box3::BottomLeftFloor);
	Vector3f U = box.corner(Box3::TopRightCeil);

	Slab slabs[3];
	slabs[0] = Slab(Vector3f(1.0f, 0.0f, 0.0f), -L[0], -U[0]);
	slabs[1] = Slab(Vector3f(0.0f, 1.0f, 0.0f), -L[1], -U[1]);
	slabs[2] = Slab(Vector3f(0.0f, 0.0f, 1.0f), -L[2], -U[2]);

	Vector3f Q = Vector3f(ray.orig.x(), ray.orig.y(), ray.orig.z());
	Vector3f D = Vector3f(ray.dir.x(), ray.dir.y(), ray.dir.z());

	float t0 = 0, t1 = FLT_MAX;
	float _t0, _t1;
	for (int i = 0; i < 3; ++i) {
		if (slabs[i].normal.dot(D)) {
			float NdotQ = slabs[i].normal.dot(Q);
			float NdotD = slabs[i].normal.dot(D);
			_t0 = -(slabs[i].d0 + NdotQ) / NdotD;
			_t1 = -(slabs[i].d1 + NdotQ) / NdotD;
			if (_t0 > _t1) {
				float temp = _t0;
				_t0 = _t1;
				_t1 = temp;
			}
		}
		else {
			float NdotQ = slabs[i].normal.dot(Q);
			float s0 = NdotQ + slabs[i].d0;
			float s1 = NdotQ + slabs[i].d1;
			if (s0 * s1 < 0.0f) {
				_t0 = 0.0f;
				_t1 = FLT_MAX;
			}
			else return FLT_MAX;
		}

		if (t0 < _t0) t0 = _t0;
		if (t1 > _t1) t1 = _t1;
	}

	if (t0 > t1) return FLT_MAX;
	else if (t0 > 0.0f) return t0;
	else if (t1 > 0.0f) return 0.0f;
	else return FLT_MAX;
}

Box3 bounding_box(const Hittable* obj) {
    return obj->bbox();
}


#endif
