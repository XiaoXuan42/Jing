#include "Octree.h"

#include <memory>
#include <queue>

#include "CoreLayer/Math/Geometry.h"
struct Octree::OctreeNode {
    AABB boundingBox;
    std::shared_ptr<OctreeNode> subNodes[8];
    std::vector<int> primIdxBuffer;
};
Octree::OctreeNode *Octree::recursiveBuild(
    const AABB &aabb, const std::vector<int> &primIdxBuffer) {
    //* 构建方法请看实验手册
    //* 要注意的一种特殊是当节点的某个子包围盒和当前节点所有物体都相交，我们就不用细分了，当前节点作为叶子节点即可。
    OctreeNode *node = new OctreeNode();
    node->boundingBox = aabb;
    if (primIdxBuffer.size() <= ocLeafMaxSize) {
        node->primIdxBuffer = primIdxBuffer;
        return node;
    }

    AABB sub_aabb[8];
    Point3f pMin = aabb.pMin, pMax = aabb.pMax;
    Vector3f hf_vLen = (pMax - pMin) / 2.0;
    const int shift[8][3] = {{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 1},
                             {1, 0, 0}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}};
    for (int i = 0; i < 8; ++i) {
        Point3f curMin = pMin;
        for (int j = 0; j < 3; ++j) {
            curMin[j] += hf_vLen[j] * shift[i][j];
        }
        Point3f curMax = curMin + hf_vLen;
        sub_aabb[i].pMin = curMin;
        sub_aabb[i].pMax = curMax;
    }

    std::vector<int> sub_primIdxBuffer[8];
    for (auto id : primIdxBuffer) {
        for (int i = 0; i < 8; ++i) {
            if (shapes[id]->getAABB().Overlap(sub_aabb[i])) {
                sub_primIdxBuffer[i].push_back(id);
            }
        }
    }

    for (int i = 0; i < 8; ++i) {
        if (sub_primIdxBuffer[i].size() > 0) {
            node->subNodes[i] = std::shared_ptr<OctreeNode>(
                recursiveBuild(sub_aabb[i], sub_primIdxBuffer[i]));
        } else {
            node->subNodes[i] = std::shared_ptr<OctreeNode>();
        }
    }
    return node;
}
void Octree::build() {
    //* 首先计算整个场景的范围
    for (const auto &shape : shapes) {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();

        boundingBox.Expand(shape->getAABB());
    }

    //* 构建八叉树
    std::vector<int> primIdxBuffer(shapes.size());
    std::iota(primIdxBuffer.begin(), primIdxBuffer.end(), 0);
    root = recursiveBuild(boundingBox, primIdxBuffer);
}

bool Octree::rayIntersect(Ray &ray, int *geomID, int *primID, float *u,
                          float *v) const {
    // 完成八叉树求交
    if (root == nullptr) {
        return false;
    }
    return node_rayIntersect(root, ray, geomID, primID, u, v);
}

bool Octree::node_rayIntersect(const OctreeNode *node, Ray &ray, int *geomID,
                               int *primID, float *u, float *v) const {
    if (node == nullptr) {
        return false;
    }
    bool hit = false;
    if (node->primIdxBuffer.size() > 0) {
        for (int i = 0; i < node->primIdxBuffer.size(); ++i) {
            int id = node->primIdxBuffer[i];
            if (shapes[id]->rayIntersectShape(ray, primID, u, v)) {
                *geomID = id;
                hit = true;
            }
        }
        return hit;
    }

    for (int i = 0; i < 8; ++i) {
        if (node->subNodes[i] &&
            node->subNodes[i]->boundingBox.RayIntersect(ray)) {
            hit |= node_rayIntersect(node->subNodes[i].get(), ray, geomID,
                                     primID, u, v);
        }
    }
    return hit;
}