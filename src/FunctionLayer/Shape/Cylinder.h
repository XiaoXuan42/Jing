#include "Shape.h"
class Cylinder : public Shape {
public:
    bool rayIntersectShape(Ray &ray, int *primID, float *u,
                           float *v) const override;

    void fillIntersection(float distance, int primID, float u, float v,
                          SurfaceIntersection *intersection) const override;

    void uniformSampleOnSurface(Vector2f sample, SurfaceIntersection *result,
                                float *pdf) const override;

    Cylinder(const Json &json);

private:
    float height;
    float radius;
    float phiMax;
};