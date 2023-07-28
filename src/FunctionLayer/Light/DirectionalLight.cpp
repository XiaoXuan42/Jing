#include "DirectionalLight.h"

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Light/Light.h"
#include "ResourceLayer/JsonUtil.h"

DirectionalLight::DirectionalLight(const Json &json) : InfiniteLight(json) {
    energy_ = fetchRequired<Spectrum>(json, "energy");
    direction_ = fetchRequired<Vector3f>(json, "direction");
}

Spectrum DirectionalLight::evaluateEmission(const Ray &ray) const {
    if (dot(direction_, ray.direction) <= -1.0f + EPSILON) {
        return energy_;
    }
    return Spectrum(0.0f);
}

LightSampleResult DirectionalLight::sample(const Point3f &shadingPoint,
                                           const Vector2f &sample) const {
    LightSampleResult result;
    result.direction = -direction_;
    result.distance = FLT_MAX;
    result.energy = energy_;
    result.pdf = 1.0f;
    result.isDelta = true;
    return result;
}