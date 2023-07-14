#pragma once
#include <CoreLayer/Math/Math.h>

#include <memory>

class Medium;
struct Ray {
    Ray() = default;

    Ray(Point3f _origin, Vector3f _direction, float _tNear = 1e-4f,
        float _tFar = 1e10f, float _time = .0f)
        : origin(_origin),
          direction(normalize(_direction)),
          tNear(_tNear),
          tFar(_tFar),
          time(_time),
          medium(nullptr) {}
    Ray(Point3f _origin, Point3f _destination, float _time = .0f)
        : origin(_origin), tNear(1e-4f), time(_time), medium(nullptr) {
        Vector3f o2d = _destination - _origin;
        tFar = o2d.length() - 1e-4f;
        direction = normalize(o2d);
    }

    Point3f at(float distance) const { return origin + distance * direction; }

    //* 基本参数
    Point3f origin;
    Vector3f direction;
    float tNear, tFar, time;

    //* 光线微分
    bool hasDifferentials = false;
    Point3f originX, originY;
    Vector3f directionX, directionY;

    Medium *medium;
};