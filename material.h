#ifndef MATERIAL_H
#define MATERIAL_H

#include "raytrace_utils.h"
#include "hittable.h"
#include "texture.h"

struct HitRecord;

class material {
public:
    Color Ks, Kt;
    double alpha, ior;
    shared_ptr<texture> Kd;
    material(shared_ptr<texture> d) : Kd(d) {}
    material(shared_ptr<texture> d, const Color& s, double a)
        : Kd(d), Ks(s), alpha(a) {}
    material(shared_ptr<texture> d, const Color& s, double a, const Color& t, double _ior)
        : Kd(d), Ks(s), Kt(t), alpha(a), ior(_ior) {}
    material(const material& rhs) :
        Kd(rhs.Kd), Ks(rhs.Ks), Kt(rhs.Kt), ior(rhs.ior), alpha(rhs.alpha) {}
    virtual bool scatter(
        const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered
    ) const = 0;
    
    virtual bool isLight() const = 0;
};

class BRDF : public material {
public:
    BRDF(shared_ptr<texture> d) : material(d) {}
    BRDF(shared_ptr<texture> d, const Color& s, double a) : material(d, s, a) {}
    BRDF(shared_ptr<texture> d, const Color& s, double a, const Color& t, double ior) : material(d, s, a, t, ior) {}

    virtual bool scatter(
        const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered
    ) const override {
        attenuation = Ks;
        return true;
    }

    bool isLight() const override { return false; }

};

class DiffuseLight : public material {
public:
    DiffuseLight(shared_ptr<texture> d) : material(d) {}

    virtual bool scatter(
        const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered
    ) const override {
        return false;
    }

    bool isLight() const override { return true; }

public:
 
};


#endif
