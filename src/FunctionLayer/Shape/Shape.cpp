#include "Shape.h"

#include <FunctionLayer/Material/Matte.h>
#include <FunctionLayer/Material/Mirror.h>

#include "CoreLayer/Math/Transform.h"
#include "FunctionLayer/Ray/Ray.h"
#include "ResourceLayer/JsonUtil.h"

Shape::Shape(const Json &json) {
    if (json.contains("transform")) {
        transform = Transform(json["transform"]);
    }
    if (json.contains("material")) {
        material = Factory::construct_class<Material>(json["material"]);
    } else {
        material = std::make_shared<MatteMaterial>();
    }

    medium_inside = nullptr;
    medium_outside = nullptr;
    if (json.contains("medium")) {
        if (json["medium"].contains("inside")) {
            medium_inside = Factory::construct_class_unique<Medium>(
                json["medium"]["inside"]);
        }
        if (json["medium"].contains("outside")) {
            medium_outside = Factory::construct_class_unique<Medium>(
                json["medium"]["outside"]);
        }
    }
}

void UserShapeBound(const RTCBoundsFunctionArguments *args) {
    Shape *shape = static_cast<Shape *>(args->geometryUserPtr);
    auto [pMin, pMax] = shape->getAABB();
    args->bounds_o->lower_x = pMin[0];
    args->bounds_o->lower_y = pMin[1];
    args->bounds_o->lower_z = pMin[2];

    args->bounds_o->upper_x = pMax[0];
    args->bounds_o->upper_y = pMax[1];
    args->bounds_o->upper_z = pMax[2];
}

void UserShapeIntersect(const RTCIntersectFunctionNArguments *args) {
    int *valid = args->valid;
    if (!valid[0]) return;

    Shape *shape = static_cast<Shape *>(args->geometryUserPtr);
    RTCRayHit *rayhit = (RTCRayHit *)args->rayhit;

    Point3f origin{rayhit->ray.org_x, rayhit->ray.org_y, rayhit->ray.org_z};
    Vector3f direction{rayhit->ray.dir_x, rayhit->ray.dir_y, rayhit->ray.dir_z};
    Ray ray{origin, direction, 1e-4f, rayhit->ray.tfar};

    float u, v;
    int primID;
    bool hit = shape->rayIntersectShape(ray, &primID, &u, &v);
    if (hit) {
        rayhit->ray.tfar = ray.tFar;
        rayhit->hit.geomID = shape->geometryID;
        rayhit->hit.primID = primID;
        rayhit->hit.u = u;
        rayhit->hit.v = v;
    }
}

void UserShapeOcclude(const RTCOccludedFunctionNArguments *args) {
    // TODO 暂不实现
}

RTCGeometry Shape::getEmbreeGeometry(RTCDevice device) const {
    RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
    rtcSetGeometryUserPrimitiveCount(geometry, 1);
    rtcSetGeometryUserData(geometry, (void *)this);
    rtcSetGeometryBoundsFunction(geometry, UserShapeBound, nullptr);
    rtcSetGeometryIntersectFunction(geometry, UserShapeIntersect);
    rtcSetGeometryOccludedFunction(geometry, UserShapeOcclude);
    rtcCommitGeometry(geometry);
    return geometry;
}