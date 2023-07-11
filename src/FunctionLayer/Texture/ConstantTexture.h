#pragma once

#include <CoreLayer/ColorSpace/Spectrum.h>

#include "Texture.h"
template <typename TReturn>
class ConstantTexture : public Texture<TReturn> {
public:
    ConstantTexture(const TReturn &_data) : data(_data) {}

    virtual TReturn evaluate(const SurfaceIntersection &intersection) const override {
        return data;
    }

    virtual TReturn evaluate(const TextureCoord &texCoord) const override {
        return data;
    }

private:
    TReturn data;
};