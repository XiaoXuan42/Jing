#pragma once

#include <memory>

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Medium/Medium.h"
#include "FunctionLayer/Medium/MediumPhase.h"
#include "FunctionLayer/Shape/Intersection.h"
#include "ResourceLayer/JsonUtil.h"

class HomoMedium : public Medium {
public:
    explicit HomoMedium(const Json &json);
    HomoMedium(std::unique_ptr<PhaseFunction> phase, const Spectrum &sigma_a,
               const Spectrum &sigma_s)
        : phase_(std::move(phase)),
          sigma_a_(sigma_a),
          sigma_s_(sigma_s),
          sigma_t_(sigma_a + sigma_s) {}

    virtual Spectrum Emission(const Point3f &p,
                              const Vector3f &dir) const override {
        return Spectrum(0.0f);
    }
    virtual Spectrum Tr(const Point3f &p, const Vector3f &dir, float t,
                        Sampler &sampler) const override;
    virtual MediumIntersection sample_forward(const Ray &ray,
                                              Sampler &sampler) const override;
    virtual MediumInScatter sample_scatter(const Point3f &p, const Vector3f &wo,
                                           Sampler &sampler) const override;
    float scatter_phase(const Vector3f &wo, const Vector3f &wi) const override;

private:
    std::unique_ptr<PhaseFunction> phase_;
    Spectrum sigma_a_, sigma_s_, sigma_t_;
};
