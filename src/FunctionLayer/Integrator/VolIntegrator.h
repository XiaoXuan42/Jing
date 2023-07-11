#pragma once

#include "Integrator.h"

class VolIntegrator : Integrator {
public:
    Spectrum li(Ray &ray, const Scene &scene,
                std::shared_ptr<Sampler> sampler) const override;
};