#pragma once

#include <openvdb/openvdb.h>

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Geometry.h"
#include "CoreLayer/Math/Transform.h"
#include "FunctionLayer/Shape/Intersection.h"
#include "Medium.h"
#include "MediumPhase.h"
#include "ResourceLayer/JsonUtil.h"

// 和Cube一样，grid默认是中心位于(0, 0,
// 0)处，棱长为2的AABB，但是采样的时候会平移放缩到[0, 1]^3
class GridMedium : public Medium {
public:
    explicit GridMedium(const Transform &transform) : transform_(transform) {}
    virtual ~GridMedium() = default;
    explicit GridMedium(const Json &json) {
        if (json.contains("transform")) {
            transform_ = Transform(json["transform"]);
        }
    }

    Vector3f toLocal(const Vector3f &d) const {
        vecmat::vec4f hlocalDir = {d[0], d[1], d[2], 0.0f};
        hlocalDir = transform_.invScale * transform_.invRotate * hlocalDir;
        return Vector3f(hlocalDir[0], hlocalDir[1], hlocalDir[2]);
    }
    Point3f toLocal(const Point3f &p) const {
        vecmat::vec4f hlocalOrigin = {p[0], p[1], p[2], 1.0f};
        hlocalOrigin = transform_.invScale * transform_.invRotate *
                       transform_.invTranslate * hlocalOrigin;
        hlocalOrigin /= hlocalOrigin[3];
        return Point3f(hlocalOrigin[0], hlocalOrigin[1], hlocalOrigin[2]);
    }
    void toSampleCoor(Point3f &p) const {
        for (int i = 0; i < 3; ++i) {
            p[i] = (p[i] + 1.0f) * 0.5f;
        }
    }

    // p in [0, 1]^3
    float Density(const Point3f &p) const { return 0.0f; }

protected:
    static float linearLerp(float a0, float a1, float d) {
        return a0 + (a1 - a0) * d;
    }

    static float triLerp(float val[2][2][2], float dx, float dy, float dz) {
        float y0z0 = linearLerp(val[0][0][0], val[1][0][0], dx);
        float y0z1 = linearLerp(val[0][0][1], val[1][0][1], dx);
        float y1z0 = linearLerp(val[0][1][0], val[1][1][0], dx);
        float y1z1 = linearLerp(val[0][1][1], val[1][1][1], dx);
        float z0 = linearLerp(y0z0, y1z0, dy);
        float z1 = linearLerp(y0z1, y1z1, dy);
        return linearLerp(z0, z1, dz);
    }

    Transform transform_;
};

template <typename T>
struct DeltaSamplingAlg {
    static_assert(std::is_base_of_v<GridMedium, T>);

    // ray is in world coordination
    static Spectrum delta_sampling_tr(const T &derived, const Point3f &p,
                                      const Vector3f &d, float invMaxDensity,
                                      float sigma_t, float tMax,
                                      Sampler &sampler) {
        float tr = 1.0f;
        float t = 0;
        Point3f origin = derived.toLocal(p);
        derived.toSampleCoor(origin);
        Vector3f dir = normalize(derived.toLocal(d));
        while (true) {
            t -= fm::log(1 - sampler.next1D()) * invMaxDensity / sigma_t;
            if (t >= tMax) {
                break;
            }
            float density = derived.Density(origin + t * dir);
            tr *= 1 - std::max(0.0f, density * invMaxDensity);
        }
        return Spectrum(tr);
    }

    static MediumIntersection delta_sampling_forward(
        const T &derived, const Ray &ray, const Spectrum &sigma_s,
        float invMaxDensity, float sigma_t, Sampler &sampler) {
        MediumIntersection mit;
        float t = ray.tNear;

        Point3f origin = derived.toLocal(ray.origin);
        derived.toSampleCoor(origin);
        Vector3f dir = normalize(derived.toLocal(ray.direction));

        while (true) {
            t -= fm::log(1 - sampler.next1D()) * invMaxDensity / sigma_t;
            if (t >= ray.tFar) {
                mit.weight = Spectrum(1.0f);
                mit.t = ray.tFar;
                mit.position = ray.at(ray.tFar);
                break;
            }
            if (derived.Density(origin + t * dir) * invMaxDensity >
                sampler.next1D()) {
                mit.weight = sigma_s / sigma_t;
                mit.t = t;
                mit.position = ray.at(t);
                break;
            }
        }
        return mit;
    }
};

class GridDensityMedium : public GridMedium {
public:
    GridDensityMedium(std::unique_ptr<PhaseFunction> phase,
                      const Transform &transform, const Spectrum &sigma_a,
                      const Spectrum &sigma_s, int nx, int ny, int nz,
                      std::unique_ptr<float[]> d)
        : GridMedium(transform),
          phase_(std::move(phase)),
          sigma_a_(sigma_a),
          sigma_s_(sigma_s),
          nx_(nx),
          ny_(ny),
          nz_(nz),
          nynz_(ny * nz),
          density_(std::move(d)) {
        Spectrum sigma_t = sigma_a_ + sigma_s_;
        sigma_t_ = sigma_t[0];
        for (int i = 1; i < Spectrum::cntChannel(); ++i) {
            if (sigma_t[i] != sigma_t[0]) {
                std::cerr << "Error: GridDensityMedium requires that sigma_t "
                             "is equal in all channels"
                          << std::endl;
                exit(-1);
            }
        }
        invMaxDensity_ = FLT_MAX;
        float maxDensity = 0.0;
        for (int i = 0; i < nx * ny * nz; ++i) {
            maxDensity = std::max(maxDensity, d[i]);
        }
        invMaxDensity_ = 1.0 / maxDensity;
    }
    explicit GridDensityMedium(const Json &json);

    Spectrum Emission(const Point3f &p, const Vector3f &dir) const override {
        return Spectrum(0.0f);
    }

    // p inside [0, 1]^3
    float Density(const Point3f &p) const;
    Spectrum Tr(const Point3f &p, const Vector3f &dir, float t,
                Sampler &sampler) const override;
    MediumIntersection sample_forward(const Ray &ray,
                                      Sampler &sampler) const override;
    MediumInScatter sample_scatter(const Point3f &p, const Vector3f &wo,
                                   Sampler &sampler) const override {
        return phase_->sample_scatter(p, wo, sampler);
    }
    float scatter_phase(const Vector3f &wo, const Vector3f &wi) const override {
        return phase_->phase(wo, wi);
    }

private:
    std::unique_ptr<PhaseFunction> phase_;

    Spectrum sigma_a_, sigma_s_;
    float sigma_t_;
    int nx_, ny_, nz_;
    int nynz_;
    std::unique_ptr<float[]> density_;
    float invMaxDensity_;
};

class VDBGridMedium : public GridMedium {
public:
    explicit VDBGridMedium(const Json &json);

    Spectrum Emission(const Point3f &p, const Vector3f &dir) const override;
    virtual Spectrum Tr(const Point3f &p, const Vector3f &dir, float tMax,
                        Sampler &sampler) const override;
    virtual MediumIntersection sample_forward(const Ray &ray,
                                              Sampler &sampler) const override;
    virtual MediumInScatter sample_scatter(const Point3f &p, const Vector3f &wo,
                                           Sampler &sampler) const override;
    virtual float scatter_phase(const Vector3f &wo,
                                const Vector3f &wi) const override {
        return phase_->phase(wo, wi);
    }
    float Density(const Point3f &p) const;

private:
    float sample_grids(const openvdb::FloatGrid::Ptr &grids,
                       const Point3f &p) const {
        float px = bboxMin_[0] + bboxLen_[0] * p[0];
        float py = bboxMin_[1] + bboxLen_[1] * p[1];
        float pz = bboxMin_[2] + bboxLen_[2] * p[2];
        int fx = std::floor(px), fy = std::floor(py), fz = std::floor(pz);
        float dx = px - fx, dy = py - fy, dz = pz - fz;
        float val[2][2][2];
        auto accessor = grids->getConstAccessor();
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    openvdb::Coord coord(fx + i, fy + j, fz + k);
                    val[i][j][k] = accessor.getValue(coord);
                }
            }
        }
        return triLerp(val, dx, dy, dz);
    }

    std::unique_ptr<PhaseFunction> phase_;

    openvdb::FloatGrid::Ptr densityGrids_;
    openvdb::FloatGrid::Ptr temperatureGrids_;
    int bboxMin_[3], bboxMax_[3];
    // 边界的voxel的中心点之间的距离
    int bboxLen_[3];
    Spectrum sigma_a_, sigma_s_;
    float sigma_t_;
    float invMaxDensity_;
    float densityScale_;

    Spectrum temperatureScale_, temperatureBias_;
};