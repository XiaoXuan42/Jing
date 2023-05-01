#pragma once

#include "CoreLayer/Math/Geometry.h"
#include "NDF.h"
#include <cmath>

class BeckmannDistribution : public NDF {
public:
  BeckmannDistribution() noexcept = default;
  virtual ~BeckmannDistribution() noexcept = default;
  virtual float getD(const Vector3f &whLocal,
                     const Vector2f &alpha) const noexcept override {
    // 根据公式即可
    // 各向同性
    // FIXME: 实现各项异性
    float a = alpha[0];
    auto w_normalize = normalize(whLocal);
    float theta = std::acos(w_normalize[1]);
    float tan_theta = std::tan(theta);
    float d = std::exp(-tan_theta * tan_theta / (a * a));
    d /= M_PI * a * a * std::pow(std::cos(theta), 4);
    return d;
  }
  // tips:
  // float getG1(...) {}
  float getG1(const Vector3f &w, const Vector2f &alpha) const noexcept {
    auto w_normalize = normalize(w);
    float theta = acos(w_normalize[1]);
    // 各向同性
    // FIXME: 实现各项异性
    float a = 1.0f / (alpha[0] * std::tan(theta));
    if (a < 1.6) {
      return (3.535 * a + 2.181 * a * a) / (1 + 2.276 * a + 2.577 * a * a);
    } else {
      return 1.0f;
    }
  }

  virtual float getG(const Vector3f &woLocal, const Vector3f &wiLocal,
                     const Vector2f &alpha) const noexcept override {
    // 根据公式即可
    // tips: return getG1(wo) * getG1(wi);
    return getG1(woLocal, alpha) * getG1(wiLocal, alpha);
  }
  virtual float pdf(const Vector3f &woLocal, const Vector3f &whLocal,
                    const Vector2f &alpha) const noexcept override {
    return getD(whLocal, alpha) * whLocal[1];
  }
  virtual Vector3f sampleWh(const Vector3f &woLocal, const Vector2f &alpha,
                            const Vector2f &sample) const noexcept override {
    float a = alpha[0];
    float tan_theta_2 = -std::log(1 - sample[0]) * a * a;
    float phi = sample[1] * 2 * PI;

    float cos_theta = std::sqrt(1.f / (1.f + tan_theta_2));
    float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
    return {sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
  }
};