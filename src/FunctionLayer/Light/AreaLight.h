#pragma once
#include <FunctionLayer/Shape/Shape.h>

#include "Light.h"
class AreaLight : public Light {
public:
    AreaLight(const Json &json);

    virtual Spectrum evaluateEmission(const SurfaceIntersection &intersection,
                                      const Vector3f &wo) const override;
    virtual LightSampleResult sample(const Point3f &shadingPoint,
                                     const Vector2f &sample) const override;

public:
    std::shared_ptr<Shape> shape;

private:
    Spectrum energy;
};