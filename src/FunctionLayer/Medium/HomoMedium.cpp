#include "HomoMedium.h"

#include <memory>

#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Geometry.h"
#include "FastMath/FastMath.h"
#include "FunctionLayer/Medium/Medium.h"
#include "ResourceLayer/Factory.h"
#include "ResourceLayer/JsonUtil.h"

Spectrum HomoMedium::Tr(const Point3f &p, const Vector3f &w, float t) {
    Spectrum tr(0.0);
    for (int i = 0; i < Spectrum::cntChannel(); ++i) {
        tr[i] = fm::exp(-t * sigma_t_[i]);
    }
    return tr;
}

void HomoMedium::sample_forward(const Ray &ray, Sampler &sampler,
                                MediumIntersection &mit) {
    float t = 0.0;
    int channelIdx = int((sampler.next1D() - 1e-5) * Spectrum::cntChannel());
    float c = sigma_t_[channelIdx];
    float dis = -fm::log(1 - sampler.next1D()) / c;

    if (dis > ray.tFar) {
        mit.distance = ray.tFar;
        mit.position = ray.at(ray.tFar);
        Spectrum tr = Tr(ray.origin, ray.direction, ray.tFar);
        // 这里利用了Tr的具体表达式, p = (tr[0] + tr[1] + tr[2]) / 3
        mit.beta = tr * 3 / (tr[0] + tr[1] + tr[2]);
    } else {
        mit.distance = dis;
        mit.position = ray.at(dis);
        Spectrum tr = Tr(ray.origin, ray.direction, dis);
        float p = 1.0f - (tr[0] + tr[1] + tr[2]) / 3.0f;
        mit.beta = tr * sigma_s_ / p;
    }
}

void HomoMedium::sample_scatter(const Point3f &p, const Vector3f &wo,
                                Sampler &sampler, MediumInScatter &mis) {
    float pdf = 1.0f;
    mis.wi = phase_->sample(wo, sampler, &pdf);
    // \int p_{scatter}L d\omega: MCS: p_{scatter}L / p(sampling)
    float scale = phase_->phase(wo, mis.wi) / pdf;
    mis.beta = Spectrum(scale);
}

float HomoMedium::scatter_phase(const Vector3f &wo, const Vector3f &wi) {
    return phase_->phase(wo, wi);
}

HomoMedium::HomoMedium(const Json &json) {
    float g = fetchRequired<float>(json, "g");
    phase_ = std::make_unique<PhaseHG>(PhaseHG(g));
    sigma_t_ = fetchRequired<Spectrum>(json, "sigma_t");
    sigma_s_ = fetchRequired<Spectrum>(json, "sigma_s");
}

REGISTER_CLASS(HomoMedium, "homomedium")
