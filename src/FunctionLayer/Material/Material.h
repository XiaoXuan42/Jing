#pragma once
#include <CoreLayer/Math/Math.h>
#include <FunctionLayer/Shape/Intersection.h>
#include <FunctionLayer/Texture/NormalTexture.h>
#include <ResourceLayer/Factory.h>
#include <ResourceLayer/JsonUtil.h>

#include "./BxDF/BSDF.h"
class Material {
public:
    Material() {
        // donothing
    }

    Material(const Json &json) {
        if (json.contains("normalmap"))
            normalMap = std::make_shared<NormalTexture>(json["normalmap"]);
    }

    virtual std::shared_ptr<BSDF> computeBSDF(
        const SurfaceIntersection &intersection) const = 0;

    void computeShadingGeometry(const SurfaceIntersection &intersection,
                                Vector3f *normal, Vector3f *tangent,
                                Vector3f *bitangent) const;

    virtual bool is_transparent() const { return false; }
    virtual bool is_empty() const { return false; }

protected:
    std::shared_ptr<NormalTexture> normalMap;
};