#pragma once

#include "FunctionLayer/Material/BxDF/Transparent.h"
#include "FunctionLayer/Shape/Intersection.h"
#include "Material.h"

class EmptyMaterial : public Material {
public:
    explicit EmptyMaterial(const Json &json) : Material(json) {}

    virtual std::shared_ptr<BSDF> computeBSDF(
        const SurfaceIntersection &its) const override {
        return std::make_shared<Transparent>(
            Transparent(its.normal, its.tangent, its.bitangent));
    }
};
