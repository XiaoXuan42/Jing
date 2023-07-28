#pragma once

#include "Light.h"

class DirectionalLight : public InfiniteLight {
public:
    explicit DirectionalLight(const Json &json);

    virtual Spectrum evaluateEmission(const Ray &ray) const override;
    virtual LightSampleResult sample(const Point3f &shadingPoint,
                                     const Vector2f &sample) const override;

private:
    Spectrum energy_;
    Vector3f direction_;
};