#pragma once
#include <FunctionLayer/Acceleration/Acceleration.h>
#include <FunctionLayer/Light/EnvironmentLight.h>
#include <FunctionLayer/Light/Light.h>
#include <ResourceLayer/JsonUtil.h>
class Scene {
public:
    Scene() = delete;

    Scene(const Json &json);

    ~Scene() = default;

    std::optional<SurfaceIntersection> rayIntersect(Ray &ray) const;
    Spectrum Tr(const Ray &ray, Sampler &sampler) const;

    std::shared_ptr<Light> sampleLight(float sample, float *pdf) const;

    float pdf(std::shared_ptr<Light> light) const;

    std::vector<std::shared_ptr<InfiniteLight>> infiniteLights;

private:
    std::shared_ptr<Acceleration> acceleration;

    Distribution<std::shared_ptr<Light>> lightDistribution;
};
