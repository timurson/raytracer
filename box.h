#ifndef BOX_H
#define BOX_H

#include "aarect.h"
#include "hittable_list.h"

class Box : public Hittable {

public:
    Box() {}
    Box(const Point3& p0, const Point3& p1, shared_ptr<material> ptr);

    virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

    Box3 bbox() const override {
        return Box3(Vector3f(min(box_min[0], box_max[0]), min(box_min[1], box_max[1]), min(box_min[2], box_max[2])),
            Vector3f(max(box_min[0], box_max[0]), max(box_min[1], box_max[1]), max(box_min[2], box_max[2])));
    }

public:
    Point3 box_min;
    Point3 box_max;
    HittableList  sides;

};

Box::Box(const Point3& p0, const Point3& p1, shared_ptr<material> ptr) {
    box_min = p0;
    box_max = p1;

    sides.add(make_shared<XY_Rect>(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), ptr));
    sides.add(make_shared<XY_Rect>(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), ptr));

    sides.add(make_shared<XZ_Rect>(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), ptr));
    sides.add(make_shared<XZ_Rect>(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), ptr));

    sides.add(make_shared<YZ_Rect>(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), ptr));
    sides.add(make_shared<YZ_Rect>(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), ptr));
}

bool Box::hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
    return sides.hit(r, t_min, t_max, rec);
}


#endif
