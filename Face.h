#ifndef FACE_H
#define FACE_H

#include "Types.h"
#define INF 1e12

class Face {
public:
    // default constructor
    Face();
    
    // initialize with specified compoenents
    Face(const Eigen::Vector3d& indices0);
    
    // indices
    Eigen::Vector3d indices;
    
    // returns face normal
    Eigen::Vector3d normal(const Mesh& mesh) const;
    
    // computes the bounding box of the face
    BoundingBox boundingBox(const Mesh& mesh) const;
    
    // computes the centroid of the face
    Eigen::Vector3d centroid(const Mesh& mesh) const;
    
    // checks if face shares edges with other faces
    bool shareEdge(const Mesh& mesh, const int fIdx) const;
    
    // checks if point is contained in face
    bool containsPoint(const Mesh& mesh, const Eigen::Vector3d& p) const;
    
    // checks if two faces overlap
    bool overlap(const Mesh& mesh, const int fIdx, const Eigen::Vector3d& normal) const;
};

#endif
