#include "GridMedium.h"

#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/ValueTransformer.h>

#include <algorithm>
#include <cmath>
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
    max_density = std::max(max_density, 1e-6f);
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

// p in [0, 1]^3
float GridDensityMedium::Density(const Point3f &p) const {
    Point3f cp;
    for (int i = 0; i < 3; ++i) {
        cp[i] = std::max(std::min(1 - EPSILON, p[i]), EPSILON);
    }
    float px = cp[0] * nx_, py = cp[1] * ny_, pz = cp[2] * nz_;
    int fx = std::floor(px), fy = std::floor(py), fz = std::floor(pz);
    float dx = px - fx, dy = py - fy, dz = pz - fz;
    float val[2][2][2];
    float *data = density_.get();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                int idx = fx + i, idy = fy + j, idz = fz + k;
                val[i][j][k] = data[idx * nynz_ + idy * nz_ + idz];
            }
        }
    }
    return triLerp(val, dx, dy, dz);
}

MediumIntersection GridDensityMedium::sample_forward(const Ray &ray,
                                                     Sampler &sampler) const {
    return DeltaSamplingAlg<GridDensityMedium>::delta_sampling_forward(
        *this, ray, sigma_s_, invMaxDensity_, sigma_t_, sampler);
}

Spectrum GridDensityMedium::Tr(const Point3f &p, const Vector3f &dir,
                               float tMax, Sampler &sampler) const {
    return DeltaSamplingAlg<GridDensityMedium>::delta_sampling_tr(
        *this, p, dir, invMaxDensity_, sigma_t_, tMax, sampler);
}

REGISTER_CLASS(GridDensityMedium, "gridDensityMedium")

VDBGridMedium::VDBGridMedium(const Json &json) : GridMedium(json) {
    float g = fetchRequired<float>(json, "g");
    phase_ = std::make_unique<PhaseHG>(g);

    densityScale_ = fetchOptional(json, "densityScale", 1.0f);

    std::string vdbPath = fetchRequired<std::string>(json, "file");
    vdbPath = FileUtil::getFullPath(vdbPath);
    openvdb::io::File file(vdbPath);
    file.open();
    openvdb::GridBase::Ptr pgrid = file.readGrid("density");
    densityGrids_ = openvdb::gridPtrCast<openvdb::FloatGrid>(pgrid);
    auto bbox = densityGrids_->evalActiveVoxelBoundingBox();
    auto bboxMin = bbox.min().asVec3i(), bboxMax = bbox.max().asVec3i();
    for (int i = 0; i < 3; ++i) {
        bboxMin_[i] = bboxMin[i];
        bboxMax_[i] = bboxMax[i];
    }

    if (file.hasGrid("temperature")) {
        openvdb::GridBase::Ptr tgrid = file.readGrid("temperature");
        temperatureGrids_ = openvdb::gridPtrCast<openvdb::FloatGrid>(tgrid);
        bbox = temperatureGrids_->evalActiveVoxelBoundingBox();
        bboxMin = bbox.min().asVec3i();
        bboxMax = bbox.max().asVec3i();
        for (int i = 0; i < 3; ++i) {
            bboxMin_[i] = std::min(bboxMin[i], bboxMin_[i]);
            bboxMax_[i] = std::max(bboxMax[i], bboxMax_[i]);
        }
    }
    temperatureScale_ = fetchOptional(json, "temperatureScale", Spectrum(1.0f));
    temperatureBias_ = fetchOptional(json, "temperatureBias", Spectrum(0.0f));

    for (int i = 0; i < 3; ++i) {
        bboxLen_[i] = bboxMax[i] - bboxMin[i];
    }

    file.close();

    float maxDensity = 1e-6f;
    openvdb::tools::foreach (
        densityGrids_->beginValueOn(),
        [&](const openvdb::FloatGrid::ValueOnIter &iter) {
            maxDensity = std::max(maxDensity, densityScale_ * iter.getValue());
        },
        false);
    invMaxDensity_ = 1.0f / maxDensity;

    sigma_a_ = fetchRequired<Spectrum>(json, "sigma_a");
    sigma_s_ = fetchRequired<Spectrum>(json, "sigma_s");
    sigma_t_ = sigma_a_[0] + sigma_s_[0];
}

Spectrum VDBGridMedium::Emission(const Point3f &p, const Vector3f &dir) const {
    if (!temperatureGrids_) {
        return Spectrum(0.0f);
    }
    Point3f sampleP = toLocal(p);
    toSampleCoor(sampleP);
    float t = sample_grids(temperatureGrids_, sampleP);
    return temperatureScale_ * (Spectrum::blackBody(t) - temperatureBias_);
}

float VDBGridMedium::Density(const Point3f &p) const {
    float px = bboxMin_[0] + bboxLen_[0] * p[0];
    float py = bboxMin_[1] + bboxLen_[1] * p[1];
    float pz = bboxMin_[2] + bboxLen_[2] * p[2];
    int fx = std::floor(px), fy = std::floor(py), fz = std::floor(pz);
    float dx = px - fx, dy = py - fy, dz = pz - fz;
    float val[2][2][2];
    auto accessor = densityGrids_->getConstAccessor();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                openvdb::Coord coord(fx + i, fy + j, fz + k);
                val[i][j][k] = accessor.getValue(coord);
            }
        }
    }
    return densityScale_ * triLerp(val, dx, dy, dz);
}

Spectrum VDBGridMedium::Tr(const Point3f &p, const Vector3f &dir, float tMax,
                           Sampler &sampler) const {
    return DeltaSamplingAlg<VDBGridMedium>::delta_sampling_tr(
        *this, p, dir, invMaxDensity_, sigma_t_, tMax, sampler);
}

MediumIntersection VDBGridMedium::sample_forward(const Ray &ray,
                                                 Sampler &sampler) const {
    return DeltaSamplingAlg<VDBGridMedium>::delta_sampling_forward(
        *this, ray, sigma_s_, invMaxDensity_, sigma_t_, sampler);
}

MediumInScatter VDBGridMedium::sample_scatter(const Point3f &p,
                                              const Vector3f &wo,
                                              Sampler &sampler) const {
    return phase_->sample_scatter(p, wo, sampler);
}

REGISTER_CLASS(VDBGridMedium, "vdbGridMedium")
