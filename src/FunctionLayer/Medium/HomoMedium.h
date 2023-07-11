#pragma once


#include "CoreLayer/ColorSpace/Spectrum.h"
#include "Medium.h"


class HomoMedium : Medium {
public:
    HomoMedium(PhaseFunction phase): Medium(phase) {}

    Spectrum Tr(const Point3f &p, const Vector3f &w, float t) override;
    MediumInteraction sample(const Ray &ray, const Intersection &its) override;
    MediumInScatter in_scatter(const Point3f &p, const Vector3f &wo, std::shared_ptr<Sampler> sampler) override;
};
