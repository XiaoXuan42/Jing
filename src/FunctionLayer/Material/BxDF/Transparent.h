#pragma once

#include "BSDF.h"
#include "CoreLayer/ColorSpace/Spectrum.h"

class Transparent : public BSDF {
public:
    Transparent(const Vector3f &_normal, const Vector3f &_tangent,
                const Vector3f &_bitangent)
        : BSDF(_normal, _tangent, _bitangent) {}

    virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const override {
        return Spectrum(.0f);
    }

    virtual BSDFSampleResult sample(const Vector3f &wo,
                                    const Vector2f &sample) const override {
        BSDFSampleResult res;
        res.pdf = 1.0f;
        res.type = BSDFType::Transmission;
        res.weight = 1.0f;
        res.wi = -wo;
        return res;
    }
};