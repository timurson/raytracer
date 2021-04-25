#ifndef TEXTURE_H
#define TEXTURE_H

#include "perlin.h"

// Disable pedantic warnings for this external library.
#ifdef _MSC_VER
    // Microsoft Visual C++ Compiler
#pragma warning (push, 0)
#endif

#define STB_IMAGE_WRITE_IMPLEMENTATION
//#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Restore warning levels.
#ifdef _MSC_VER
    // Microsoft Visual C++ Compiler
#pragma warning (pop)
#endif

class texture {
public:
    virtual Color value(double u, double v, const Point3& p) const = 0;
};

class solid_color : public texture {
public:
    solid_color() {}
    solid_color(Color c) : colorValue(c) {}

    virtual Color value(double u, double v, const Vec3& p) const override {
        return colorValue;
    }

private:
    Color colorValue;
};

class checker_texture : public texture {
public:
    checker_texture() {}

    checker_texture(shared_ptr<texture> _even, shared_ptr<texture> _odd)
        : even(_even), odd(_odd) {}

    checker_texture(Color c1, Color c2, double sc)
        : even(make_shared<solid_color>(c1)), odd(make_shared<solid_color>(c2)), scale(sc) {}

    virtual Color value(double u, double v, const Vec3& p) const override {
        double odd_even = floor(p.x() * scale) + floor(p.y() * scale) + floor(p.z() * scale);
        odd_even = odd_even / 2.0 - int(odd_even / 2.0);
        odd_even *= 2;
        if (odd_even)
            return odd->value(u, v, p);
        else
            return even->value(u, v, p);
    }

public:
    shared_ptr<texture> odd;
    shared_ptr<texture> even;
    double scale;
};

class noise_texture : public texture {
public:
    noise_texture() {}
    noise_texture(double sc) : scale(sc) {}

    virtual Color value(double u, double v, const Vec3& p) const override {
        //return Color(1,1,1)*0.5*(1 + noise.turb(scale * p));
        //return Color(1,1,1)*noise.turb(scale * p); // scale 4.5
        return Color(1, 1, 1) * 0.5 * (1 + sin(scale * p.z() + 30 * noise.turb(p)));
    }

public:
    perlin noise;
    double scale;
};

class image_texture : public texture {
public:
    const static int bytes_per_pixel = 3;

    image_texture()
        : data(nullptr), width(0), height(0), bytes_per_scanline(0) {}

    image_texture(const char* filename) {
        auto components_per_pixel = bytes_per_pixel;

        data = stbi_load(
            filename, &width, &height, &components_per_pixel, components_per_pixel);

        if (!data) {
            std::cerr << "ERROR: Could not load texture image file '" << filename << "'.\n";
            width = height = 0;
        }

        bytes_per_scanline = bytes_per_pixel * width;
    }

    ~image_texture() {
        free(data);
    }

    virtual Color value(double u, double v, const Vec3& p) const override {
        // If we have no texture data, then return solid cyan as a debugging aid.
        if (data == nullptr)
            return Color(0, 1, 1);

        // Clamp input texture coordinates to [0,1] x [1,0]
        u = clamp(u, 0.0, 1.0);
        v = 1.0 - clamp(v, 0.0, 1.0);  // Flip V to image coordinates

        auto i = static_cast<int>(u * width);
        auto j = static_cast<int>(v * height);

        // Clamp integer mapping, since actual coordinates should be less than 1.0
        if (i >= width)  i = width - 1;
        if (j >= height) j = height - 1;

        const auto color_scale = 1.0 / 255.0;
        auto pixel = data + j * bytes_per_scanline + i * bytes_per_pixel;

        return Color(color_scale * pixel[0], color_scale * pixel[1], color_scale * pixel[2]);
    }

private:
    unsigned char* data;
    int width, height;
    int bytes_per_scanline;
};

#endif
