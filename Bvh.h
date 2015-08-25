#ifndef BVH_H
#define BVH_H

#include "Types.h"
#include "BoundingBox.h"

struct Node {
    // member variables
    BoundingBox boundingBox;
    int startId, range, rightOffset;
};

class Bvh {
public:
    Bvh(Mesh *meshPtr0, const int leafSize0 = 1);
    
    // checks if a face overlaps with another face. Returns face id
    int doesOverlap(const int fid, const Eigen::Vector3d& normal) const;
    
private:    
    // builds the bvh
    void build();
    
    int nodeCount, leafCount, leafSize;
    std::vector<Node> flatTree;
    Mesh *meshPtr;
};

#endif
