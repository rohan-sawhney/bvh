#ifndef BVH_H
#define BVH_H

#include "Types.h"

class Node;

class Bvh {
public:
    Bvh(Mesh *meshPtr0);
    
    // checks if a face overlaps with other faces
    bool doesOverlap(const Face& f);
    
private:
    // computes the bounding box a face
    BoundingBox computeBoundingBox(const int fId);
    
    // computes the centroid a face
    Eigen::Vector3d computeCentroid(const int fId);
    
    // builds the bvh
    void build();
    
    int nodeCount, leafCount;
    std::vector<Node> flatTree;
    Mesh *meshPtr;
};

#endif
