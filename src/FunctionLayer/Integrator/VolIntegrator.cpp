#include "VolIntegrator.h"

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <vector>

#include "CoreLayer/ColorSpace/RGB.h"
#include "CoreLayer/ColorSpace/Spectrum.h"
#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Integrator/Integrator.h"
#include "FunctionLayer/Light/Light.h"
#include "FunctionLayer/Material/BxDF/BSDF.h"
#include "FunctionLayer/Material/Material.h"
#include "FunctionLayer/Medium/Medium.h"
#include "FunctionLayer/Ray/Ray.h"
#include "FunctionLayer/Shape/Intersection.h"
#include "ResourceLayer/JsonUtil.h"

Spectrum directLighting(const Scene &scene, const Intersection &its,
                        LightSampleResult &res, float sample_pdf,
                        Medium *medium, Sampler &sampler);

Spectrum VolIntegrator::li(Ray &ray, const Scene &scene,
                           std::shared_ptr<Sampler> sampler) const {
    auto itsOpt = scene.rayIntersect(ray);
    Spectrum L(0.0f);

    Spectrum throughput(1.0f);
    /*
    如果是specular，那么brdf就是一个delta函数，在tracing过程中统计光源的影响的时候就会落下，
    因此需要对这种情况特殊处理。
    */
    bool specularBounce = false;
    Medium *medium = ray.medium;

    int depth = 0;
    while (true) {
        if (!itsOpt.has_value()) {
            for (auto light : scene.infiniteLights) {
                Spectrum tr = scene.Tr(ray, *sampler);
                L += throughput * tr * light->evaluateEmission(ray);
            }
            break;
        }
        const Vector3f wo = -ray.direction;
        Intersection its;
        SurfaceIntersection sit = itsOpt.value();
        its = sit;

        if (depth == 0 || specularBounce) {
            auto light = sit.shape->light;
            if (light) {
                Spectrum lighting = light->evaluateEmission(sit, wo);
                Ray shadowRay = ray;
                shadowRay.tFar -= EPSILON;
                Spectrum tr = scene.Tr(shadowRay, *sampler);
                L += throughput * tr * lighting;
            }
        }

        ++depth;
        if (depth >= maxDepth) {
            break;
        }

        bool hit = true;
        MediumIntersection mit;
        medium = ray.medium;
        if (medium) {
            mit = medium->sample_forward(ray, *sampler);
            throughput *= mit.weight;

            if (mit.t < ray.tFar) {
                hit = false;
                its = mit;
            }
        }

        std::shared_ptr<BSDF> bsdf = nullptr;
        if (hit) {
            bsdf = sit.shape->material->computeBSDF(sit);
        }

        // 计算光源的贡献
        for (auto light : scene.infiniteLights) {
            auto res = light->sample(its, sampler->next2D());
            Medium *init_medium = nullptr;
            Vector3f wi = res.direction;
            if (hit) {
                init_medium = sit.getMedium(wi);
            } else {
                init_medium = medium;
            }
            Spectrum lightSpec =
                directLighting(scene, its, res, 1.0f, init_medium, *sampler);
            if (!lightSpec.isZero()) {
                if (hit) {
                    Spectrum f = bsdf->f(wo, wi);
                    L += throughput * f * lightSpec;
                } else {
                    float p = medium->scatter_phase(wo, wi);
                    L += throughput * p * lightSpec;
                }
            }
        }
        float pdfLight = .0f;
        auto light = scene.sampleLight(sampler->next1D(), &pdfLight);
        if (pdfLight > 0 && light) {
            auto res = light->sample(its, sampler->next2D());
            Medium *init_medium = nullptr;
            Vector3f wi = res.direction;
            if (hit) {
                init_medium = sit.getMedium(wi);
            } else {
                init_medium = medium;
            }
            Spectrum lightSpec = directLighting(scene, its, res, pdfLight,
                                                init_medium, *sampler);
            if (!lightSpec.isZero()) {
                if (hit) {
                    Spectrum f = bsdf->f(wo, wi);
                    L += throughput * f * lightSpec;
                } else {
                    float p = medium->scatter_phase(wi, wo);
                    L += throughput * p * lightSpec;
                }
            }
        }

        // ruassian roulette
        if (depth > 5 && sampler->next1D() > roulette_) {
            break;
        }
        throughput /= roulette_;

        // 下一步光线的方向
        if (hit) {
            auto bsdf_sample_result = bsdf->sample(wo, sampler->next2D());
            if (bsdf_sample_result.weight.isZero()) break;

            throughput *= bsdf_sample_result.weight;
            ray = Ray(sit.position, bsdf_sample_result.wi);
            ray.medium = sit.getMedium(ray.direction);

            specularBounce = bsdf_sample_result.type == BSDFType::Specular;
        } else {
            // emission
            L += throughput * medium->Emission(mit.position, wo);
            // scatter
            MediumInScatter mis =
                medium->sample_scatter(mit.position, wo, *sampler);
            throughput *= mis.weight;

            ray = Ray(mit.position, mis.wi);
            ray.medium = medium;

            specularBounce = false;
        }
        itsOpt = scene.rayIntersect(ray);
    }

    return L;
}

Spectrum directLighting(const Scene &scene, const Intersection &its,
                        LightSampleResult &res, float sample_pdf,
                        Medium *medium, Sampler &sampler) {
    Ray ray(its.position, res.direction, 1e-4f, res.distance);
    ray.medium = medium;

    Spectrum tr = scene.Tr(ray, sampler);
    Spectrum spec(0.0f);
    if (!tr.isZero()) {
        res.pdf *= sample_pdf;
        float pdf = convertPDF(res, its);
        spec = tr * res.energy / pdf;
    }
    return spec;
}

VolIntegrator::VolIntegrator(const Json &json) {
    maxDepth = fetchRequired<uint32_t>(json, "maxDepth");
    if (json.contains("roulette")) {
        roulette_ = fetchRequired<float>(json, "roulette");
    } else {
        roulette_ = 1.0f;
    }
}

REGISTER_CLASS(VolIntegrator, "vol")
