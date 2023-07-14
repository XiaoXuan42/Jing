#pragma once

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Shape/Intersection.h"
#include "Medium.h"
#include "ResourceLayer/JsonUtil.h"

#include <memory>

class HomoMedium : public Medium {
public:
    explicit HomoMedium(const Json &json);
    HomoMedium(std::unique_ptr<PhaseFunction> phase, const Spectrum &sigma_t,
               const Spectrum &sigma_s)
        : phase_(std::move(phase)), sigma_t_(sigma_t), sigma_s_(sigma_s) {}

    Spectrum Tr(const Point3f &p, const Vector3f &w, float t) override;
    void sample_forward(const Ray &ray, Sampler &sampler,
                MediumIntersection &mit) override;
    void sample_scatter(const Point3f &p, const Vector3f &wo, Sampler &sampler,
                    MediumInScatter &mis) override;
    float scatter_phase(const Vector3f &wo, const Vector3f &wi) override;

private:
    std::unique_ptr<PhaseFunction> phase_;
    Spectrum sigma_t_, sigma_s_;
};
