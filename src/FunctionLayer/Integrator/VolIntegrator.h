#pragma once

#include "Integrator.h"

class VolIntegrator : public Integrator {
public:
    VolIntegrator() = default;

    VolIntegrator(const Json &json);

    Spectrum li(Ray &ray, const Scene &scene,
                std::shared_ptr<Sampler> sampler) const override;

private:
    uint32_t maxDepth;
};