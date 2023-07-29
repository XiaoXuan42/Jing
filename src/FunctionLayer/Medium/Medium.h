#pragma once

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Sampler/Sampler.h"
#include "FunctionLayer/Shape/Intersection.h"

struct MediumInScatter {
    Vector3f wi;
    Spectrum weight;
};

class Medium {
public:
    virtual Spectrum Emission(const Point3f &p, const Vector3f &dir) const = 0;
    virtual Spectrum Tr(const Point3f &p, const Vector3f &dir, float t,
                        Sampler &sampler) const = 0;
    virtual MediumIntersection sample_forward(const Ray &ray,
                                              Sampler &sampler) const = 0;
    virtual MediumInScatter sample_scatter(const Point3f &p, const Vector3f &wo,
                                           Sampler &sampler) const = 0;
    virtual float scatter_phase(const Vector3f &wo,
                                const Vector3f &wi) const = 0;
};
