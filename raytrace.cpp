//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <chrono> 


#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"
#include "hittable_list.h"
#include "sphere.h"
#include "box.h"
#include "cylinder.h"
#include "triangle.h"
#include "torus.h"
#include "minimizer.h"
#include "camera.h"
#include "material.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <Eigen_unsupported/Eigen/BVH>

// Global collection of raytraced objects
HittableList world;
Camera cam(Point3(0, 0, 0), Point3(0, 0, -1), Vec3(0, 1, 0), 22.6175, 4.0 / 3.0);
Color currentDiffuse, currentSpecular, currentTransmission;
double roughness, indexOfRefraction;

#define RUSSIAN_ROULETTE 0.8
#define EPSILON 0.000001

#define USE_BVH

Color Scene::TracePath(const Ray& r, KdBVH<float, 3, Hittable*>& tree)
{
    Color color(0, 0, 0);
    Color weight(1, 1, 1);
    Minimizer minimizer(r);

    BVMinimize(tree, minimizer);
    HitRecord record = minimizer.minInt;
    if (record.t < FLT_MAX) {

        if (!record.mat_ptr->isLight()) {

            double MIS = 1.0;
            Vec3 omega_o = -r.direction();
            while (random_double() < RUSSIAN_ROULETTE) {

                Vec3 N = record.normal.normalized();
                Color Kd = record.mat_ptr->Kd->value(record.u, record.v, record.p);
                Color Ks = record.mat_ptr->Ks;
                Color Kt = record.mat_ptr->Kt;

                // calculate the probabilities
                double KdNorm = Vec3(Kd.x(), Kd.y(), Kd.z()).norm();
                double KsNorm = Vec3(Ks.x(), Ks.y(), Ks.z()).norm();
                double KtNorm = Vec3(Kt.x(), Kt.y(), Kt.z()).norm();
                double S = KdNorm + KsNorm + KtNorm;

                double probDiffuse = KdNorm / S;
                double probSpecular = KsNorm / S;
                double probTransmission = KtNorm / S;

                Point3 pos = record.p;

                // Explicit light connection 
                HitRecord expLight = SampleLight();
                double pdfLight = PdfLight(expLight) / GeometryFactor(record, expLight);  // Probability of L, converted to angular measure
                // make a shadow ray
                Ray ray(pos, expLight.p - pos);

                minimizer = Minimizer(ray);
                BVMinimize(tree, minimizer);
                HitRecord shadowRec = minimizer.minInt;
                if (pdfLight > 0.0 && shadowRec.t < FLT_MAX && shadowRec.obj_ptr == expLight.obj_ptr) {
                    Vec3 omega_i = (expLight.p - pos).normalized();
                    // Prob the explicit light could be chosen implicitly
                    double pdfBrdf = PdfBrdf(record, omega_o, omega_i, probDiffuse, probSpecular, probTransmission) * RUSSIAN_ROULETTE;
                    MIS = pdfLight * pdfLight / (pdfLight * pdfLight + pdfBrdf * pdfBrdf);
                    Color f = EvalScattering(record, omega_o, omega_i, shadowRec.t);
                    color += weight * MIS * f / pdfLight * expLight.mat_ptr->Kd->value(shadowRec.u, shadowRec.v, shadowRec.p);
                }
    
                // Extend the path
                ray = Ray(pos, SampleBrdf(record, omega_o, probDiffuse, probSpecular)); // Choose a sample direction from P
                HitRecord lastRec = record; // make a copy of current record
                minimizer = Minimizer(ray);
                BVMinimize(tree, minimizer);
                record = minimizer.minInt;
                if (record.t < FLT_MAX) {
                    Vec3 omega_i = ray.dir.normalized();
                    Color f = EvalScattering(lastRec, omega_o, omega_i, record.t);
                    double pdfBrdf = PdfBrdf(lastRec, omega_o, omega_i, probDiffuse, probSpecular, probTransmission) * RUSSIAN_ROULETTE;
                    if (pdfBrdf < EPSILON) break;
                    weight = weight.cwiseProduct(f / pdfBrdf);

                    // Implicit light connection
                    if (record.mat_ptr->isLight()) {
                        // Prob the implicit light could be chosen explicitly
                        double pdfLight = PdfLight(record) / GeometryFactor(lastRec, record);
                        MIS = pdfBrdf * pdfBrdf / (pdfLight * pdfLight + pdfBrdf * pdfBrdf);
                        color += weight * MIS * record.mat_ptr->Kd->value(record.u, record.v, record.p);
                        break;
                    }
                    omega_o = -omega_i;
                }
                else{
                    break; // no intersection found
                }
               
            } // end while loop
        }
        else {
            color = record.mat_ptr->Kd->value(record.u, record.v, record.p);
        }
    }

    return color;
}

Scene::Scene() 
{ 
    realtime = new Realtime();
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    realtime->triangleMesh(mesh); 
    // create a triangle mesh for ray tracer
    auto brdf_mat = make_shared<BRDF>(currentTex, currentSpecular, roughness, currentTransmission, indexOfRefraction);
    //world.add(make_shared<TriangleMesh>(make_shared<MeshData>(*mesh), brdf_mat));

    for (auto&& tri : mesh->triangles) {
#ifdef USE_BVH
        auto triangle = new Triangle(tri.x(), tri.y(), tri.z(), brdf_mat, mesh);
        /*auto triangle = new Triangle(Point3(0.975, 0.0, 0.4),
            Point3(0.975, 0.6, 0.4), Point3(0.975, 0.4, 0.8), brdf_mat);*/
        objects.push_back(triangle);

#else
        world.add(make_shared<Triangle>(tri.x(), tri.y(), tri.z(), brdf_mat, mesh));
       /* world.add(make_shared<Triangle>(Point3(0.975, 0.0, 0.4),
            Point3(0.975, 0.6, 0.4), Point3(0.975, 0.4, 0.8), brdf_mat));*/

#endif
    }
}

Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}

////////////////////////////////////////////////////////////////////////
// Material: encapsulates surface properties
void Material::setTexture(const std::string path)
{
    int width, height, n;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* image = stbi_load(path.c_str(), &width, &height, &n, 0);

    // Realtime code below:  This sends the texture in *image to the graphics card.
    // The raytracer will not use this code (nor any features of OpenGL nor the graphics card).
    glGenTextures(1, &texid);
    glBindTexture(GL_TEXTURE_2D, texid);
    glTexImage2D(GL_TEXTURE_2D, 0, n, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 100);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, (int)GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_LINEAR_MIPMAP_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    stbi_image_free(image);
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        realtime->setScreen(int(f[1]),int(f[2]));
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        realtime->setCamera(Vector3f(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]); 

        // converting ry to degrees for our raytracing camera
        double fov = 2.0 * radians_to_degrees(atan(f[4]));
        Vector3f viewDir = realtime->ViewDirection();
        Point3 lookfrom = Point3(f[1], f[2], f[3]);
        Point3 lookat = lookfrom + Vec3(viewDir.x(), viewDir.y(), viewDir.z());
        cam = Camera(lookfrom, lookat, Vec3(0, 0, 1), fov, (double)width / height);
    }
    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        realtime->setAmbient(Vector3f(f[1], f[2], f[3])); 
    }
    else if (c == "checker") {
        currentTex = make_shared<checker_texture>(Color(f[1], f[2], f[3]), Color(f[4], f[5], f[6]), f[7]);
    }
    else if (c == "perlin") {
        currentTex = make_shared<noise_texture>(f[1]);
    }
    else if (c == "image") {
        currentTex = make_shared<image_texture>(strings[1].c_str());
    }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]);

        currentDiffuse = Color(f[1], f[2], f[3]);
        currentTex = make_shared<solid_color>(currentDiffuse);
        currentSpecular = Color(f[4], f[5], f[6]);
        roughness = f[7];
        currentTransmission = Color(f[8], f[9], f[10]);
        indexOfRefraction = f[11];
    }
    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(Vector3f(f[1], f[2], f[3])); 
        currentDiffuse = Color(f[1], f[2], f[3]);
        currentTex = make_shared<solid_color>(currentDiffuse);
    }
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        realtime->sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);

        if (currentMat->isLight()) {
            // add light sphere
            auto light = make_shared<Sphere>(Point3(f[1], f[2], f[3]), f[4], make_shared<DiffuseLight>(currentTex));
            world.add(light);
            emitters.push_back(light.get());
        }
        else {
            // add regular sphere
            world.add(make_shared<Sphere>(Point3(f[1], f[2], f[3]), f[4], make_shared<BRDF>(currentTex, currentSpecular, roughness, currentTransmission, indexOfRefraction)));
        }
    }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        realtime->box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);

        // add box for raytracing 
        auto brdf_mat = make_shared<BRDF>(currentTex, currentSpecular, roughness, currentTransmission, indexOfRefraction);
        world.add(make_shared<Box>(Point3(f[1], f[2], f[3]), Point3(f[1] + f[4], f[2] + f[5], f[3] + f[6]), brdf_mat));
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        realtime->cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat); 

        // add cylinder for raytracing
        auto brdf_mat = make_shared<BRDF>(currentTex, currentSpecular, roughness, currentTransmission, indexOfRefraction);
        world.add(make_shared<Cylinder>(Point3(f[1], f[2], f[3]), Vec3(f[4], f[5], f[6]), f[7], brdf_mat));
    }

    else if (c == "torus") {
        auto brdf_mat = make_shared<BRDF>(currentTex, currentSpecular, roughness, currentTransmission, indexOfRefraction);
        auto torus = make_shared<Torus>(f[4], f[5], brdf_mat);
        world.add(make_shared<Translate>(torus, Vec3(f[1], f[2], f[3])));
    }

    else if (c == "rotateb") {
        auto brdf_mat = make_shared<BRDF>(currentTex, currentSpecular, roughness, currentTransmission, indexOfRefraction);
        auto box = make_shared<Box>(Point3(0.0, 0.0, 0.0), Point3(f[4], f[5], f[6]), brdf_mat);
        auto rot = make_shared<RotateZ>(box, f[7]);
        world.add(make_shared<Translate>(rot, Vec3(f[1], f[2], f[3])));
    }

    else if (c == "rotates") {
        auto brdf_mat = make_shared<BRDF>(currentTex, currentSpecular, roughness, currentTransmission, indexOfRefraction);
        auto sphere = make_shared<Sphere>(Point3(0.0, 0.0, 0.0), f[4], brdf_mat);
        auto rot = make_shared<RotateZ>(sphere, f[5]);
        world.add(make_shared<Translate>(rot, Vec3(f[1], f[2], f[3])));
    }

    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        Matrix4f modelTr = translate(Vector3f(f[2],f[3],f[4]))
                          *scale(Vector3f(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);
    }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}



void Scene::TraceImage(Color* image, const int pass)
{
    realtime->run();                          // Remove this (realtime stuff)

    const int MAX_PASS = 20000;

    // Using time point and system_clock 
    auto start = std::chrono::high_resolution_clock::now();


    for (const auto& obj : world.objects) {
        objects.push_back(obj.get());
    }

    KdBVH<float, 3, Hittable*> tree(objects.begin(), objects.end());

    for (int pass = 1; pass <= MAX_PASS; ++pass) {
        
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) {

            fprintf(stderr, "Pass %i, Scanlines remaining %4d\r", pass, height - y - 1);
            for (int x = 0; x < width; x++) {

                auto u = double(x + random_double()) / (width - 1);
                auto v = double(y + random_double()) / (height - 1);
                Ray r = cam.getRay(u, v);
                
                Color pixel_color = TracePath(r, tree);
                writeColor(image, x, y, pixel_color, 1);

            } // x loop
        } // y loop

        // occasionally write image out 
        if (pass == 1 || pass == 8 || pass == 64 || pass == 512 || pass == 4096 || pass == 8192 || pass == 10000
            || pass == 15000 || pass == 20000) {
            char filename[32];
            sprintf(filename, "image_pass_%d.hdr", pass);
            std::string hdrName = filename;
            // Write the image
            WriteHdrImage(hdrName, width, height, image, pass);

        }

    } // pass loop

    fprintf(stderr, "\n");

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_mills = end - start;
    printf("Elapsed time (seconds) =  %.3f\r", elapsed_mills.count() / 1000);
}

void Scene::writeColor(Color* image, int x, int y, const Color& pixel_color, int samples_per_pixel)
{
    auto r = pixel_color.x();
    auto g = pixel_color.y();
    auto b = pixel_color.z();

    if (r != r) r = 0.0;
    if (g != g) g = 0.0;
    if (b != b) b = 0.0;

    auto scale = 1.0 / samples_per_pixel;
    r = scale * r;
    g = scale * g;
    b = scale * b;

    image[y * width + x] += Color(r, g, b);
}

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void Scene::WriteHdrImage(const std::string outName, const int width, const int height, Color* image, int pass) {

    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width * height * 3];
    float* dp = data;
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            Color pixel = image[y * width + x] / pass;
            *dp++ = pixel[0];
            *dp++ = pixel[1];
            *dp++ = pixel[2];
        }
    }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = { 0 };

    FILE* fp = fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width, height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);

    delete data;

}

HitRecord Scene::SampleLight()
{
    int index = random_int(0, emitters.size()-1);
    return SampleSphere(dynamic_cast<Sphere*>(emitters[index]));
}

HitRecord Scene::SampleSphere(Sphere* sphere) 
{
    HitRecord record;

    if (!sphere) return record;

    auto z = 2.0 * random_double() - 1;
    auto r = sqrt(1 - z * z);
    auto a = 2.0 * pi * random_double();
    record.normal = Vec3(r * cos(a), r * sin(a), z);
    record.p = sphere->center + record.normal * sphere->radius;
    record.mat_ptr = sphere->mat_ptr;
    record.obj_ptr = sphere;

    return record;
}

double Scene::PdfLight(HitRecord& record)
{
    return 1.0 / (emitters.size() * record.obj_ptr->area);
}

double Scene::PdfBrdf(HitRecord rec, Vec3& wo, Vec3& wi, double probDiff, double probSpec, double probTrans)
{
    // diffuse
    auto Pd = fabs(wi.dot(rec.normal)) / pi;

    // reflection
    Vec3 m = (wo + wi).normalized();
    auto Pr = DistributionPhong(rec, m) * fabs(m.dot(rec.normal)) / 4.0 / fabs(wi.dot(m));

    // transmission
    double etai = rec.mat_ptr->ior;
    double etao = 1.0;
    if (wo.dot(rec.normal) > 0) {
        etao = etai; etai = 1.0;
    }
    float eta = etai / etao;
    m = -(etao * wi + etai * wo).normalized();
    double woDotM = wo.dot(m);
    double r = 1.0f - eta * eta * (1 - woDotM * woDotM);
    double Pt;
    if (r < 0) Pt = Pr;
    else {
        double denom = etao * wi.dot(m) + etai * wo.dot(m);
        Pt = DistributionPhong(rec, m) * fabs(m.dot(rec.normal)) * etao * etao * fabs(wi.dot(m)) / denom / denom;
    }

    return probDiff * Pd + probSpec * Pr + probTrans * Pt;
}

double Scene::GeometryFactor(HitRecord& A, HitRecord& B)
{
    Vec3 D = A.p - B.p;
    auto result = A.normal.dot(D) * B.normal.dot(D) / (D.dot(D) * D.dot(D));
    return abs(result);
}

Vec3 Scene::SampleLobe(Vec3 N, double theta, double phi)
{
    auto sTheta = sqrt(1 - theta * theta);
    Vec3 K = Vec3(sTheta * cos(phi), sTheta * sin(phi), theta); // Vector centered around Z-axis
    Quaterniond q = Quaterniond::FromTwoVectors(Vec3::UnitZ(), N);
    return q._transformVector(K);
}

Vec3 Scene::SampleBrdf(HitRecord& record, Vec3& wo, double probDiff, double probSpec)
{
    auto randNum = random_double();
    if (randNum < probDiff){
        return SampleLobe(record.normal, sqrt(random_double()), 2.0 * pi * random_double());
    }
    else if (randNum < probDiff + probSpec){
        Vec3 m = SampleLobe(record.normal, pow(random_double(), 1.0 / (record.mat_ptr->alpha + 1.0)), 2.0 * pi * random_double());
        return 2.0 * wo.dot(m) * m - wo;
    }
    else {
        Vec3 m = SampleLobe(record.normal, pow(random_double(), 1.0 / (record.mat_ptr->alpha + 1.0)), 2.0 * pi * random_double());
        double eta = record.mat_ptr->ior;
        if (wo.dot(record.normal) > 0) eta = 1.0 / eta; // Path leaves an object, passing from inside to outside
        double woDotM = wo.dot(m);
        double r = 1.0 - eta * eta * (1 - woDotM * woDotM);
        if (r < 0) return 2.0 * woDotM * m - wo;
        else return (eta * woDotM - (wo.dot(record.normal) > 0.0f ? 1.0 : -1.0) * sqrt(r))* m - eta * wo;
    }
}

Color Scene::EvalScattering(HitRecord rec, Vec3& wo, Vec3& wi, double woT)
{
    Vec3 N = rec.normal.normalized();
    auto jacobDet = fabs(wi.dot(N) * wo.dot(N));

    // Diffuse
    Color Ed = rec.mat_ptr->Kd->value(rec.u, rec.v, rec.p) / pi;

    // Specular
    Vec3 m = (wo + wi).normalized();
    Color Er = DistributionPhong(rec, m) * G_Phong(rec, wo, wi, m) * Fresnel(rec, wi.dot(m)) / (4.0 * jacobDet);

    // Transmission
    double etai = rec.mat_ptr->ior;
    double etao = 1.0;
    if (wo.dot(rec.normal) > 0) {
        etao = etai; etai = 1.0;
    }
    double eta = etai / etao;
    m = -(etao * wi + etai * wo).normalized();
    double woDotM = wo.dot(m);
    double r = 1.0f - eta * eta * (1 - woDotM * woDotM);
    Color Et;
    if (r < 0.0) Et = Er;
    else {
        double denom = etao * wi.dot(m) + etai * wo.dot(m);
        double result = DistributionPhong(rec, m)
            * G_Phong(rec, wo, wi, m)
            / jacobDet
            * fabs(wi.dot(m) * wo.dot(m)) * etao * etao
            / denom / denom;
        Et = result * (Color(1.0, 1.0, 1.0) - Fresnel(rec, wi.dot(m))); 
    }

    Color attenuation = Color(1.0);
    if (wi.dot(rec.normal) < 0.0) {  // path enters an object, need to attenuate
        Color Kt = rec.mat_ptr->Kt;
        for (int i = 0; i < 3; ++i) attenuation[i] = exp(woT*log(Kt[i]));
    }

    return fabs(N.dot(wi)) * (Ed + Er + Et.cwiseProduct(attenuation));
}

double Scene::DistributionPhong(HitRecord& rec, Vec3& m)
{
    auto alpha = rec.mat_ptr->alpha;
    auto mDotN = m.dot(rec.normal);
    if (mDotN <= 0) return 0.0;
    return (alpha + 2.0) / 2.0 / pi * pow(mDotN, alpha);
}

double Scene::G1(HitRecord& rec, Vec3& v, Vec3& n)
{
    auto vDotN = v.dot(rec.normal);
    if (vDotN > 1.0) return 1.0;
    if (v.dot(n) / vDotN <= 0) return 0.0;
    else {
        auto tanTheta = sqrt(1.0f - vDotN * vDotN) / vDotN;
        if (tanTheta < epsilon) return 1.0;
        auto a = sqrt(rec.mat_ptr->alpha / 2.0 + 1.0) / tanTheta;
        if (a > 1.6) return 1.0;
        else return (3.535f * a + 2.181f * a * a) / (1.0 + 2.276 * a + 2.577 * a * a);
    }
}

Color Scene::Fresnel(HitRecord& rec, double d)
{
    Color Ks = rec.mat_ptr->Ks;
    return Ks + (Color(1.0) - Ks) * pow(1 - fabs(d), 5.0);
}

