#include "HomoMedium.h"
#include "CoreLayer/ColorSpace/Spectrum.h"
#include "FunctionLayer/Medium/Medium.h"
#include "FunctionLayer/Shape/Intersection.h"


MediumInteraction HomoMedium::sample(const Ray &ray, const Intersection &its) {
    return {};
}

Spectrum HomoMedium::Tr(const Point3f &p, const Vector3f &w, float t) {
    return Spectrum(0.0f);
}

MediumInScatter HomoMedium::in_scatter(const Point3f &p, const Vector3f &wo, std::shared_ptr<Sampler> sampler) {
    return {};
}
