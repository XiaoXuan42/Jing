#pragma once

#include "Medium.h"
#include "ResourceLayer/JsonUtil.h"

class GridMedium : public Medium {
public:
    explicit GridMedium(const Json &json);

    Spectrum Tr(const Point3f &p, const Vector3f &w, float t) override;
    void sample_forward(const Ray &ray, Sampler &sampler,
                        MediumIntersection &mit) override;
    void sample_scatter(const Point3f &p, const Vector3f &wo, Sampler &sampler,
                        MediumInScatter &mis) override;
    float scatter_phase(const Vector3f &wo, const Vector3f &wi) override;
};