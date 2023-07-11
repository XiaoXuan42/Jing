#include "Shape.h"
class Cone : public  Shape{
public:
    Cone(const Json & json);
    bool rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const override;

    void fillIntersection(float distance, int primID, float u, float v, SurfaceIntersection *intersection) const override;

    void uniformSampleOnSurface(Vector2f sample, SurfaceIntersection *result, float *pdf) const override;

private:
    float phiMax;
    float radius;
    float height;
    float cosTheta;
};