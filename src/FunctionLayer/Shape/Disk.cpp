#include "Disk.h"

#include <chrono>

#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include "CoreLayer/Math/Transform.h"
#include "ResourceLayer/Factory.h"
bool Disk::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    Point3f origin = ray.origin;
    Vector3f direction = ray.direction;
    vecmat::vec4f o{origin[0], origin[1], origin[2], 1.f},
        d{direction[0], direction[1], direction[2], 0.f};
    o = transform.invRotate * transform.invTranslate * o;
    d = transform.invRotate * transform.invTranslate * d;
    o /= o[3];
    origin = Point3f{o[0], o[1], o[2]};
    direction = Vector3f{d[0], d[1], d[2]};

    if (std::abs(direction[2]) <= EPSILON) {
        return false;
    }
    float t = origin[2] / -direction[2];
    if (t < 0 || t <= ray.tNear || t >= ray.tFar) {
        return false;
    }
    Point3f intersect = origin + direction * t;
    float r =
        std::sqrt(intersect[0] * intersect[0] + intersect[1] * intersect[1]);
    if (r > radius || r < innerRadius) {
        return false;
    }
    double ang = std::atan2(intersect[1], intersect[0]);
    if (ang < 0) {
        ang += 2 * M_PI;
    }
    if (ang > phiMax) {
        return false;
    }
    ray.tFar = t;

    *u = ang / phiMax;
    *v = (r - innerRadius) / (radius - innerRadius);
    return true;
}

void Disk::fillIntersection(float distance, int primID, float u, float v,
                            SurfaceIntersection *intersection) const {
    intersection->shape = this;
    intersection->t = distance;
    intersection->texCoord = Vector2f{u, v};

    Point3f position;
    double ang = phiMax * u;
    double r = v * (radius - innerRadius) + innerRadius;
    position[0] = cos(ang) * r;
    position[1] = sin(ang) * r;
    intersection->position = transform.toWorld(position);

    Vector3f normal{0.f, 0.f, 1.f};
    intersection->normal = transform.toWorld(normal);

    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

Disk::Disk(const Json &json) : Shape(json) {
    radius = fetchOptional(json, "radius", 1.f);
    innerRadius = fetchOptional(json, "inner_radius", 0.f);
    phiMax = fetchOptional(json, "phi_max", 2 * PI);
    AABB local(Point3f(-radius, -radius, 0), Point3f(radius, radius, 0));
    boundingBox = transform.toWorld(local);
}

void Disk::uniformSampleOnSurface(Vector2f sample, SurfaceIntersection *result,
                                  float *pdf) const {
    // 采样光源 暂时不用实现
}
REGISTER_CLASS(Disk, "disk")
