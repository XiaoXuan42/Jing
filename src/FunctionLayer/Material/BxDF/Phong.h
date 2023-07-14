#pragma once
#include <math.h>

#include "BSDF.h"
#include "Warp.h"

class PhongReflection : public BSDF {
public:
    PhongReflection(const Vector3f &_normal, const Vector3f &_tangent,
                    const Vector3f &_bitangent, Spectrum _kd, Spectrum _ks,
                    float _p)
        : BSDF(_normal, _tangent, _bitangent), kd(_kd), ks(_ks), p(_p) {}

    virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const override {
        // 1. 转换坐标系到局部坐标
        // 2. 根据公式计算 K_d, K_s
        // 3. return K_d + K_s
        // tips:
        // Phong模型brdf实现不包括环境光项；其I/r^2项包含在光源采样步骤中，因此brdf中不包含I/r^2。
        Spectrum diffuse{0.f};
        Spectrum specular{0.f};
        Vector3f woLocal = toLocal(wo), wiLocal = toLocal(wi);
        woLocal = normalize(woLocal);
        wiLocal = normalize(wiLocal);
        diffuse = std::max(wiLocal[1], 0.0f) * kd;
        Vector3f wrLocal = 2 * wiLocal[1] * Vector3f{0., 1., 0.} - wiLocal;
        if (wrLocal[1] > 0) {
            float cos_theta = dot(wrLocal, woLocal);
            if (cos_theta > 0) {
                specular = powf(cos_theta, p) * ks;
            }
        }
        return diffuse + specular;
    }

    float pdf(const Vector3f &wo, const Vector3f &wi) const {
        Vector3f wiLocal = toLocal(wi);
        return squareToCosineHemispherePdf(wiLocal);
    }

    virtual BSDFSampleResult sample(const Vector3f &wo,
                                    const Vector2f &sample) const override {
        Vector3f wiLocal = squareToCosineHemisphere(sample);
        auto wi = toWorld(wiLocal);
        auto bsdf_f = f(wo, wi);
        auto bsdf_pdf = pdf(wo, wi);
        return {bsdf_f / bsdf_pdf, wi, bsdf_pdf, BSDFType::Diffuse};
    }

private:
    Spectrum kd;  // 漫反射系数
    Spectrum ks;  // 高光（镜面反射）系数
    float p;      // 高光衰减系数
};