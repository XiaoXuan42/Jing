#include "GridDensityMedium.h"

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Shape/Intersection.h"

float GridDensityMedium::triLearp(const Point3f &pGrid) const {
    int fx = std::floor(pGrid[0]), fy = std::floor(pGrid[1]),
        fz = std::floor(pGrid[2]);
    int cx = fx + 1, cy = fy + 1, cz = fz + 1;
    float dx = pGrid[0] - fx, dy = pGrid[1] - fy, dz = pGrid[2] - fz;
    float y0z0 =
        dx * queryDensity(cx, fy, fz) + (1 - dx) * queryDensity(fx, fy, fz);
    float y0z1 =
        dx * queryDensity(cx, fy, cz) + (1 - dx) * queryDensity(fx, fy, cz);
    float y1z0 =
        dx * queryDensity(cx, cy, fz) + (1 - dx) * queryDensity(fx, cy, fz);
    float y1z1 =
        dx * queryDensity(cx, cy, cz) + (1 - dx) * queryDensity(fx, cy, cz);
    float z0 = dy * y1z0 + (1 - dy) * y0z0;
    float z1 = dy * y1z1 + (1 - dy) * y0z1;
    float val = dz * z1 + (1 - dz) * z0;
    return val;
}

float GridDensityMedium::Density(const Point3f &p) const {
    const Point3f pGrid = Point3f(p[0] * nx_, p[1] * ny_, p[2] * nz_);
    return triLearp(pGrid);
}

MediumIntersection GridDensityMedium::sample_forward(const Ray &ray,
                                                     Sampler &sampler) {
    MediumIntersection mit;
    float t = ray.tNear;

    while (true) {
        t -= fm::log(1 - sampler.next1D()) * invMaxDensity_ / sigma_t_;
        if (t >= ray.tFar) {
            mit.weight = Spectrum(1.0f);
            mit.t = ray.tFar;
            mit.position = ray.at(ray.tFar);
            break;
        }
        if (Density(ray.at(t)) * invMaxDensity_ > sampler.next1D()) {
            mit.weight = sigma_s_ / sigma_t_;
            mit.t = t;
            mit.position = ray.at(t);
            break;
        }
    }
    return mit;
}

Spectrum GridDensityMedium::Tr(const Point3f &p, const Vector3f &w, float tMax,
                               Sampler &sampler) {
    float tr = 1.0f;
    float t = 0;
    while (true) {
        t -= fm::log(1 - sampler.next1D()) * invMaxDensity_ / sigma_t_;
        if (t >= tMax) {
            break;
        }
        float density = Density(p + w * t);
        tr *= 1 - std::max(0.0f, density * invMaxDensity_);
    }
    return Spectrum(tr);
}