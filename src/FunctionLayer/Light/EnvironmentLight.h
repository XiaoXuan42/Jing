#pragma once
#include <CoreLayer/Math/Distribution.h>
#include <FunctionLayer/Ray/Ray.h>
#include <FunctionLayer/Texture/ImageTexture.h>

#include "CoreLayer/Math/Geometry.h"
#include "Light.h"
class EnvironmentLight : public InfiniteLight {
public:
    EnvironmentLight() = delete;

    explicit EnvironmentLight(const Json &json);

    Spectrum evaluateEmission(const Ray &ray) const override;

    virtual LightSampleResult sample(const Point3f &shadingPoint,
                                     const Vector2f &sample) const override;

private:
    std::shared_ptr<Texture<Spectrum>> environmentMap;
    Distribution<Vector2i> energyDistribution;
};