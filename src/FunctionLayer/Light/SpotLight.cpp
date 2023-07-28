#include "SpotLight.h"

#include <ResourceLayer/Factory.h>

#include "CoreLayer/Math/Geometry.h"

SpotLight::SpotLight(const Json &json) : Light(json) {
    position_ = fetchRequired<Point3f>(json, "position");
    energy_ = fetchRequired<Spectrum>(json, "energy");
    type = LightType::SpotLight;
}

//! 由于点光源不会与光线发生相交，故该函数实际上不会被调用
Spectrum SpotLight::evaluateEmission(const SurfaceIntersection &intersection,
                                     const Vector3f &wo) const {
    return Spectrum(.0f);
}

LightSampleResult SpotLight::sample(const Point3f &p,
                                    const Vector2f &sample) const {
    Vector3f shadingPoint2sample = position_ - p;
    return LightSampleResult{energy_,
                             normalize(shadingPoint2sample),
                             shadingPoint2sample.length() - EPSILON,
                             Vector3f(),
                             1.f,
                             true,
                             type};
}

REGISTER_CLASS(SpotLight, "spotLight")