#pragma once

#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Transform.h"
#include "FunctionLayer/Shape/Intersection.h"
#include "Medium.h"
#include "ResourceLayer/JsonUtil.h"

class GridDensityMedium : public Medium {
public:
    GridDensityMedium(const Transform &transform, const Spectrum &sigma_a,
                      const Spectrum &sigma_s, int nx, int ny, int nz,
                      std::unique_ptr<float[]> d)
        : sigma_a_(sigma_a),
          sigma_s_(sigma_s),
          nx_(nx),
          ny_(ny),
          nz_(nz),
          nynz_(ny * nz),
          transform_(transform),
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

    // 为了和cube shape保持一致，transform后的坐标是在[-1.0, 1.0]^3区间的。
    Point3f toGrid(const Point3f &p) const {
        auto cube_coor = transform_.toLocal(p);
        for (int i = 0; i < 3; ++i) {
            cube_coor[i] = (cube_coor[i] + 1.0f) * 0.5f;
            cube_coor[i] = std::max(EPSILON, cube_coor[i]);
            cube_coor[i] = std::min(1.0f - EPSILON, cube_coor[i]);
        }
        return cube_coor;
    }
    float queryDensity(int i, int j, int k) const {
        return density_.get()[i * nynz_ + j * nz_ + k];
    }
    float triLearp(const Point3f &pGrid) const;
    // p inside [0, 1]^3
    float Density(const Point3f &p) const;
    Spectrum Tr(const Point3f &p, const Vector3f &w, float t,
                Sampler &sampler) override;
    MediumIntersection sample_forward(const Ray &ray,
                                      Sampler &sampler) override;
    MediumInScatter sample_scatter(const Point3f &p, const Vector3f &wo,
                                   Sampler &sampler) override;
    float scatter_phase(const Vector3f &wo, const Vector3f &wi) override;

private:
    const Spectrum sigma_a_, sigma_s_;
    float sigma_t_;
    const int nx_, ny_, nz_;
    const int nynz_;
    Transform transform_;
    std::unique_ptr<float[]> density_;
    float invMaxDensity_;
};
