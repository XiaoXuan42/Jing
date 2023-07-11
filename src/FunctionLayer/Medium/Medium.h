#pragma once

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Sampler/Sampler.h"
#include "FunctionLayer/Shape/Intersection.h"

class PhaseFunction {
public:
    float sample(const Vector3f &wo, const Vector3f &wi,
                 std::shared_ptr<Sampler> sampler) const {
        return 0.25 * INV_PI;
    };
    float p(const Vector3f &wo, const Vector3f &wi);
};

struct MediumInScatter {
    Vector3f wi;
    Spectrum beta;
};

class Medium {
public:
    Medium(PhaseFunction phase) : phase(phase) {}

    virtual Spectrum Tr(const Point3f &p, const Vector3f &w, float t) = 0;
    virtual void sample(const Ray &ray, const SurfaceIntersection &its,
                        MediumIntersection &interaction) = 0;
    virtual void in_scatter(const Point3f &p, const Vector3f &wo,
                            std::shared_ptr<Sampler> sampler,
                            MediumInScatter &mis) = 0;

    PhaseFunction phase;
};