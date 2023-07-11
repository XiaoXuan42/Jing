#pragma once

#include <ResourceLayer/Image.h>

#include "Texture.h"

class NormalTexture : public Texture<Vector3f> {
public:
    NormalTexture() = delete;

    NormalTexture(const Json &json);

    virtual Vector3f evaluate(const SurfaceIntersection &intersection) const override;

    virtual Vector3f evaluate(const TextureCoord &texCoord) const override;

private:
    std::shared_ptr<Image> normalmap;
};