#include "AreaLight.h"

#include <ResourceLayer/Factory.h>

AreaLight::AreaLight(const Json &json) : Light(json) {
    type = LightType::AreaLight;
    shape = Factory::construct_class<Shape>(json["shape"]);
    energy_ = fetchRequired<Spectrum>(json, "energy");
}

Spectrum AreaLight::evaluateEmission(const SurfaceIntersection &intersection,
                                     const Vector3f &wo) const {
    return energy_;
}

LightSampleResult AreaLight::sample(const Point3f &p,
                                    const Vector2f &sample) const {
    SurfaceIntersection sampleResult;
    float pdf;
    shape->uniformSampleOnSurface(sample, &sampleResult, &pdf);
    Vector3f shadingPoint2sample = sampleResult.position - p;

    return {energy_,
            normalize(shadingPoint2sample),
            shadingPoint2sample.length() - EPSILON,
            sampleResult.normal,
            pdf,
            false,
            type};
}

REGISTER_CLASS(AreaLight, "areaLight")