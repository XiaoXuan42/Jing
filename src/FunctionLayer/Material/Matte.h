#pragma once
#include "Material.h"

class MatteMaterial : public Material {
public:
    MatteMaterial();

    MatteMaterial(const Json &json);

    virtual std::shared_ptr<BSDF> computeBSDF(
        const SurfaceIntersection &intersection) const override;

private:
    std::shared_ptr<Texture<Spectrum>> albedo;
};