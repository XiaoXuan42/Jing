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
    int channelIdx = int((sampler.next1D() - 1e-5) * Spectrum::cntChannel());
    float c = sigma_t_[channelIdx];
    float dis = -fm::log(1 - sampler.next1D()) / c;

    if (dis > ray.tFar) {
        mit.distance = ray.tFar;
        mit.position = ray.at(ray.tFar);
        Spectrum tr = Tr(ray.origin, ray.direction, ray.tFar);
        Spectrum prob = tr;
        float p = 0.0f;
        for (int i = 0; i < Spectrum::cntChannel(); ++i) {
            p += prob[i];
        }
        p /= Spectrum::cntChannel();
        mit.beta = tr / p;
    } else {
        mit.distance = dis;
        mit.position = ray.at(dis);
        Spectrum tr = Tr(ray.origin, ray.direction, dis);
        Spectrum prob = tr * sigma_t_;
        float p = 0.0f;
        for (int i = 0; i < Spectrum::cntChannel(); ++i) {
            p += prob[i];
        }
        p /= Spectrum::cntChannel();
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
    sigma_a_ = fetchRequired<Spectrum>(json, "sigma_a");
    sigma_s_ = fetchRequired<Spectrum>(json, "sigma_s");
    sigma_t_ = sigma_a_ + sigma_s_;
}

REGISTER_CLASS(HomoMedium, "homomedium")
