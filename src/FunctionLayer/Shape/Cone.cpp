#include "Cone.h"

#include "CoreLayer/Math/Function.h"
#include "CoreLayer/Math/Geometry.h"
#include "FastMath/VecMat.h"
#include "ResourceLayer/Factory.h"

bool Cone::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.联立方程求解
    //* 3.检验交点是否在圆锥范围内
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.

    vecmat::vec4f o = {ray.origin[0], ray.origin[1], ray.origin[2], 1.0};
    vecmat::vec4f d = {ray.direction[0], ray.direction[1], ray.direction[2],
                       0.0};
    o = transform.invRotate * transform.invTranslate * o;
    d = transform.invRotate * transform.invTranslate * d;
    o /= o[3];
    Point3f origin = {o[0], o[1], o[2]};
    Vector3f direction = {d[0], d[1], d[2]};

    float costheta2 = height * height / (radius * radius + height * height);
    float A = (costheta2 - direction[2] * direction[2]);
    Point3f hat = {0, 0, height};
    Vector3f o_d = origin - hat;
    float B = 2 * (costheta2 * dot(o_d, direction) -
                   (origin[2] - height) * direction[2]);
    float C =
        costheta2 * dot(o_d, o_d) - (origin[2] - height) * (origin[2] - height);

    float tnear, tfar;
    if (!Quadratic(A, B, C, &tnear, &tfar)) {
        return false;
    }

    // side: 0: outside to inside, 1: inside to outside
    auto test_set = [&](const float t, int side) {
        if (t < 0 || t < ray.tNear || t >= ray.tFar) {
            return false;
        }
        const Point3f &p = origin + t * direction;
        if (p[2] < 0 || p[2] > height) {
            return false;
        }
        double ang = atan2(p[1], p[0]);
        if (ang < 0) {
            ang += 2 * M_PI;
        }
        if (ang > phiMax) {
            return false;
        }

        *primID = side;
        *u = ang / phiMax;
        *v = p[2] / height;
        return true;
    };

    if (test_set(tnear, 0)) {
        return true;
    }
    return test_set(tfar, 1);
}

void Cone::fillIntersection(float t, int primID, float u, float v,
                            SurfaceIntersection *intersection) const {
    /// ----------------------------------------------------
    //* 填充圆锥相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------

    intersection->shape = this;
    intersection->t = t;
    intersection->texCoord = Vector2f{u, v};

    Point3f position;
    double ang = u * phiMax;
    double h = v * height;
    double r = (height - h) / height * radius;
    position[0] = r * cos(ang);
    position[1] = r * sin(ang);
    position[2] = h;
    intersection->position = transform.toWorld(position);

    Vector3f normal{
        position[0], position[1],
        radius * radius / (height * height) * (height - position[2])};
    normal = normal / normal.length();
    // FIXME: inside or outside?
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

void Cone::uniformSampleOnSurface(Vector2f sample, SurfaceIntersection *result,
                                  float *pdf) const {}

Cone::Cone(const Json &json) : Shape(json) {
    radius = fetchOptional(json, "radius", 1.f);
    height = fetchOptional(json, "height", 1.f);
    phiMax = fetchOptional(json, "phi_max", 2 * PI);
    float tanTheta = radius / height;
    cosTheta = sqrt(1 / (1 + tanTheta * tanTheta));
    // theta = fetchOptional(json,)
    AABB localAABB =
        AABB(Point3f(-radius, -radius, 0), Point3f(radius, radius, height));
    boundingBox = transform.toWorld(localAABB);
    boundingBox = AABB(Point3f(-100, -100, -100), Point3f(100, 100, 100));
}

REGISTER_CLASS(Cone, "cone")
