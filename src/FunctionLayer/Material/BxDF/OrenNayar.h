#pragma once
#include "BSDF.h"
#include "Warp.h"
#include <cmath>
#include <cstdio>

class OrenNayarBSDF : public BSDF {
public:
  OrenNayarBSDF(const Vector3f &_normal, const Vector3f &_tangent,
                const Vector3f &_bitangent, Spectrum _albedo, float _sigma)
      : BSDF(_normal, _tangent, _bitangent), albedo(_albedo), sigma(_sigma) {}

  virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const override {
    // 1. 转换坐标系到局部坐标
    // 2. 计算 A, B, \alpha, \beta（可以直接求\sin\alpha,\tan\beta）,
    // \cos(\phi_i-\phi_o)
    // 3. return Oren-Nayar brdf
    Vector3f woLocal = toLocal(wo), wiLocal = toLocal(wi);
    float theta_o = acos(woLocal[1]), theta_i = acos(wiLocal[1]);
    float alpha = std::max(theta_o, theta_i), beta = std::min(theta_o, theta_i);
    float A = 1 - sigma * sigma / (2 * (sigma * sigma + 0.33));
    float B = 0.45 * sigma * sigma / (sigma * sigma + 0.09);
    float phi_i = atan2(wiLocal[0], wiLocal[2]), phi_o = atan2(woLocal[0], woLocal[2]);
    float coeff = 1.0 / M_PI * (A + B * std::max(0.0f, std::cos(phi_i - phi_o)) * std::sin(alpha) * std::tan(beta));
    return coeff * albedo;
  }

  virtual BSDFSampleResult sample(const Vector3f &wo,
                                  const Vector2f &sample) const override {
    Vector3f wi = squareToCosineHemisphere(sample);
    float pdf = squareToCosineHemispherePdf(wi);
    return {albedo, toWorld(wi), pdf, BSDFType::Diffuse};
  }

private:
  Spectrum albedo;
  float sigma;
};