#pragma once

#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Medium/Medium.h"
#include "FunctionLayer/Sampler/Sampler.h"

class PhaseFunction {
public:
    virtual Vector3f sample(const Vector3f &wo, Sampler &sampler,
                            float *pdf) const = 0;
    virtual float phase(const Vector3f &wo, const Vector3f &wi) const = 0;
    MediumInScatter sample_scatter(const Point3f &p, const Vector3f &wo,
                                   Sampler &sampler) const {
        MediumInScatter mis;
        float pdf = 1.0f;
        mis.wi = sample(wo, sampler, &pdf);
        float scale = phase(wo, mis.wi) / pdf;
        mis.weight = Spectrum(scale);
        return mis;
    }
};

class PhaseHG : public PhaseFunction {
public:
    explicit PhaseHG(float _g) : g(_g) {}
    Vector3f sample(const Vector3f &wo, Sampler &sampler,
                    float *pdf) const override {
        // ref: pbrt
        // cos\theta = \frac{1}{2g}((\frac{1-g^2}{-2gx + g + 1})^2 - 1 - g^2)
        // sampling according to phase function
        float cosVal = 1.0f;
        float rx = sampler.next1D();
        if (std::abs(g) < 1e-3) {
            cosVal = 1.0f - 2 * rx;
        } else {
            float tmp1 = (1 - g * g) / (1 + g - 2 * g * rx);
            cosVal = (tmp1 * tmp1 - 1 - g * g) / (2 * g);
        }

        Vector3f a1, a2;
        if (std::abs(wo[1]) < 1e-3 && std::abs(wo[2]) < 1e-3) {
            a1 = normalize(Vector3f(-wo[1], wo[0], 0));
        } else {
            a1 = normalize(Vector3f(0, -wo[2], wo[1]));
        }
        a2 = cross(wo, a1);
        float theta = 2 * PI * sampler.next1D();
        float len = fm::sqrt(1 - cosVal * cosVal);
        Vector3f wi =
            fm::cos(theta) * len * a1 + fm::sin(theta) * len * a2 + cosVal * wo;

        if (pdf) {
            *pdf = phase(wo, wi);
        }

        return wi;
    }

    float phase(const Vector3f &wo, const Vector3f &wi) const override {
        float cos = dot(wo, wi);
        float g_sq = g * g;
        float denom = 1 + g_sq + 2 * g * cos;
        return 0.25 * INV_PI * (1 - g_sq) / (denom * fm::sqrt(denom));
    }

private:
    float g;
};