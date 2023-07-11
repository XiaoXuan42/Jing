#pragma once
#include "CoreLayer/Math/Geometry.h"
#include "Light.h"

class SpotLight : public Light {
public:
    SpotLight() = delete;

    SpotLight(const Json &json);

    virtual Spectrum evaluateEmission(const SurfaceIntersection &intersection,
                                      const Vector3f &wo) const;

    virtual LightSampleResult sample(const Point3f &p,
                                     const Vector2f &sample) const;

private:
    Point3f position;
    Spectrum energy;
};