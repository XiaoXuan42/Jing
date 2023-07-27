#include "GridMedium.h"

#include <openvdb/Types.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Medium/Medium.h"
#include "FunctionLayer/Shape/Intersection.h"
#include "ResourceLayer/Factory.h"
#include "ResourceLayer/FileUtil.h"
#include "ResourceLayer/JsonUtil.h"

GridDensityMedium::GridDensityMedium(const Json &json) : GridMedium(json) {
    float g = fetchRequired<float>(json, "g");
    phase_ = std::make_unique<PhaseHG>(g);

    std::string file = fetchRequired<std::string>(json, "file");
    file = FileUtil::getFullPath(file);
    std::ifstream in;
    in.open(file, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "Error: Can't open " << file;
        exit(-1);
    }
    int nx, ny, nz;
    in.read((char *)(&nx), sizeof(int));
    in.read((char *)(&ny), sizeof(int));
    in.read((char *)(&nz), sizeof(int));
    std::unique_ptr<float[]> data = std::make_unique<float[]>(nx * ny * nz);
    in.read((char *)(data.get()), nx * ny * nz * sizeof(float));
    in.close();
    nx_ = nx;
    ny_ = ny;
    nz_ = nz;
    nynz_ = ny * nz;
    density_ = std::move(data);
    float max_density =
        *std::max_element(density_.get(), density_.get() + nx * ny * nz);
    if (max_density == 0) {
        max_density = 0.0001;
    }
    invMaxDensity_ = 1.0f / max_density;

    sigma_a_ = fetchRequired<Spectrum>(json, "sigma_a");
    sigma_s_ = fetchRequired<Spectrum>(json, "sigma_s");
    sigma_t_ = sigma_a_[0] + sigma_s_[0];
    for (int i = 1; i < Spectrum::cntChannel(); ++i) {
        if (sigma_t_ != sigma_a_[i] + sigma_s_[i]) {
            std::cerr << "Error: Sigma_t of GridDensityMedium must be equal in "
                         "all channels"
                      << std::endl;
            exit(-1);
        }
    }
}

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

// p in [0, 1]^3
float GridDensityMedium::Density(const Point3f &p) const {
    Point3f cp;
    for (int i = 0; i < 3; ++i) {
        cp[i] = std::max(std::min(1 - EPSILON, p[i]), EPSILON);
    }
    const Point3f pGrid = Point3f(cp[0] * nx_, cp[1] * ny_, cp[2] * nz_);
    return triLearp(pGrid);
}

MediumIntersection GridDensityMedium::sample_forward(const Ray &ray,
                                                     Sampler &sampler) {
    MediumIntersection mit;
    float t = ray.tNear;

    Point3f localOrigin = toLocal(ray.origin);
    toSampleCoor(localOrigin);
    Vector3f localDir = toLocal(ray.direction);

    while (true) {
        t -= fm::log(1 - sampler.next1D()) * invMaxDensity_ / sigma_t_;
        if (t >= ray.tFar) {
            mit.weight = Spectrum(1.0f);
            mit.t = ray.tFar;
            mit.position = ray.at(ray.tFar);
            break;
        }
        if (Density(localOrigin + t * localDir) * invMaxDensity_ >
            sampler.next1D()) {
            mit.weight = sigma_s_ / sigma_t_;
            mit.t = t;
            mit.position = ray.at(t);
            break;
        }
    }
    return mit;
}

Spectrum GridDensityMedium::Tr(const Point3f &p, const Vector3f &dir,
                               float tMax, Sampler &sampler) {
    float tr = 1.0f;
    float t = 0;
    Point3f localOrigin = toLocal(p);
    toSampleCoor(localOrigin);
    Vector3f localDir = toLocal(dir);
    while (true) {
        t -= fm::log(1 - sampler.next1D()) * invMaxDensity_ / sigma_t_;
        if (t >= tMax) {
            break;
        }
        float density = Density(localOrigin + localDir * t);
        tr *= 1 - std::max(0.0f, density * invMaxDensity_);
    }
    return Spectrum(tr);
}

REGISTER_CLASS(GridDensityMedium, "gridDensityMedium")

VDBGridMedium::VDBGridMedium(const Json &json) : GridMedium(json) {
    std::string vdbPath = fetchRequired<std::string>(json, "file");
    vdbPath = FileUtil::getFullPath(vdbPath);
    openvdb::io::File file(vdbPath);
    file.open();
    densityGrids_ = file.readGrid("density");
    file.close();
    bbox_ = densityGrids_->evalActiveVoxelBoundingBox();
}

Spectrum VDBGridMedium::Tr(const Point3f &p, const Vector3f &dir, float t,
                           Sampler &sampler) {
    return Spectrum(0.0f);
}

MediumIntersection VDBGridMedium::sample_forward(const Ray &ray,
                                                 Sampler &sampler) {
    return MediumIntersection();
}

MediumInScatter VDBGridMedium::sample_scatter(const Point3f &p,
                                              const Vector3f &wo,
                                              Sampler &sampler) {
    return MediumInScatter();
}

float VDBGridMedium::scatter_phase(const Vector3f &wo, const Vector3f &wi) {
    return 1.0f;
}

REGISTER_CLASS(VDBGridMedium, "vdbGridMedium")
