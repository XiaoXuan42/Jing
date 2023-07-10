#pragma once

#include <cmath>

#include "CoreLayer/Math/Geometry.h"
#include "NDF.h"

class GGXDistribution : public NDF {
public:
    GGXDistribution() noexcept = default;
    virtual ~GGXDistribution() noexcept = default;
    virtual float getD(const Vector3f &whLocal,
                       const Vector2f &alpha) const noexcept override {
        // 根据公式即可
        float a = alpha[0];
        auto w_normalize = normalize(whLocal);
        float theta = std::acos(w_normalize[1]);
        float div1 = std::pow(std::cos(theta), 4);
        float div2 = std::pow(a * a + std::tan(theta) * std::tan(theta), 2);
        return a * a / (M_PI * div1 * div2);
    }
    // tips:
    // float getG1(...) {}

    float getG1(const Vector3f &w, const Vector2f &alpha) const noexcept {
        auto w_normalize = normalize(w);
        float theta = std::acos(w_normalize[1]);
        float a = alpha[0];
        return 2.0f /
               (1.0 + std::sqrt(1 + a * a * std::tan(theta) * std::tan(theta)));
    }
    virtual float getG(const Vector3f &woLocal, const Vector3f &wiLocal,
                       const Vector2f &alpha) const noexcept override {
        // 根据公式即可
        // tips: return getG1(wo) * getG1(wi);
        return getG1(wiLocal, alpha) * getG1(woLocal, alpha);
    }
    virtual float pdf(const Vector3f &woLocal, const Vector3f &whLocal,
                      const Vector2f &alpha) const noexcept override {
        return getD(whLocal, alpha) * whLocal[1];
    }
    virtual Vector3f sampleWh(const Vector3f &woLocal, const Vector2f &alpha,
                              const Vector2f &sample) const noexcept override {
        float a = alpha[0];
        float tan_theta_2 = a * a * sample[0] / (1.f - sample[0]);
        float phi = sample[1] * 2 * PI;

        float cos_theta = std::sqrt(1.f / (1.f + tan_theta_2));
        float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
        return {sin_theta * std::cos(phi), sin_theta * std::sin(phi),
                cos_theta};
    }
};