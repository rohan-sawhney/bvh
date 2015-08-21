#ifndef FACE_H
#define FACE_H

#include "Types.h"

class Face {
public:
    // default constructor
    Face();
    
    // initialize with specified compoenents
    Face(const Eigen::Vector3d& indices0);
    
    // indices
    Eigen::Vector3d indices;
    
    BoundingBox boundingBox(const Mesh& mesh) const;
    
    Eigen::Vector3d centroid(const Mesh& mesh) const;
    
    bool shareEdge(const Mesh& mesh, const int fIdx) const;
    
    bool overlap(const Mesh& mesh, const int fIdx, Eigen::Vector3d& normal) const;
};

#endif
