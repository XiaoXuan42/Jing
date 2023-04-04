#include "Cylinder.h"
#include "CoreLayer/Math/Geometry.h"
#include "CoreLayer/Math/Function.h"
#include "FastMath/VecMat.h"
#include "ResourceLayer/Factory.h"

bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.联立方程求解
    //* 3.检验交点是否在圆柱范围内
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.
    Point3f origin = ray.origin;
    Vector3f direction = ray.direction;
    vecmat::vec4f o = {origin[0], origin[1], origin[2], 1.0};
    vecmat::vec4f d = {direction[0], direction[1], direction[2], 0.0};
    o = transform.invRotate * transform.invTranslate * o;
    d = transform.invRotate * transform.invTranslate * d;
    o /= o[3];
    origin = {o[0], o[1], o[2]};
    direction = {d[0], d[1], d[2]};

    float tnear = 0, tfar = 0;
    float A = direction[0] * direction[0] + direction[1] * direction[1];
    float B = 2 * direction[0] * origin[0] + 2 * direction[1] * origin[1];
    float C = origin[0] * origin[0] + origin[1] * origin[1] - radius * radius;
    if (!Quadratic(A, B, C, &tnear, &tfar)) {
        return false;
    }

    // side: 0: outside to inside, 1: inside to outside
    auto point_test_set = [&](float t, int side) {
        if (t < 0 || t < ray.tNear || t >= ray.tFar) {
            return false;
        }
        const Point3f pos = origin + t * direction;
        if(pos[2] < 0 || pos[2] > height) {
            return false;
        }
        double ang = atan2(pos[1], pos[0]);
        if (ang < 0) {
            ang += 2 * M_PI;
        }
        if (ang > phiMax) {
            return false;
        }

        *primID = side;
        *u = ang / phiMax;
        *v = pos[2] / height;
        ray.tFar = t;
        return true;
    };

    // tnear?
    if (point_test_set(tnear, 0)) {
        return true;
    }
    // tfar?
    return point_test_set(tfar, 1);
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------
    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};

    double ang = u * phiMax;
    Point3f position;
    position[0] = radius * cos(ang);
    position[1] = radius * sin(ang);
    position[2] = v * height;
    intersection->position = transform.toWorld(position);

    Vector3f normal = {float(cos(ang)), float(sin(ang)), 0.0f};
    /*
    if (primID > 0) {
        normal = -normal;
    }
    */
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

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")
