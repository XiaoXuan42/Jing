#pragma once

#include <CoreLayer/ColorSpace/Spectrum.h>
#include <ResourceLayer/Image.h>

#include "Mipmap.h"
#include "Texture.h"
class ImageTexture : public Texture<Spectrum> {
public:
    ImageTexture() = delete;

    ImageTexture(const Json &json);

    virtual Spectrum evaluate(const SurfaceIntersection &intersection) const override;

    virtual Spectrum evaluate(const TextureCoord &texCoord) const override;

private:
    std::shared_ptr<MipMap> mipmap;
};
