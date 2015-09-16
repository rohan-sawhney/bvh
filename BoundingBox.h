#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "Types.h"

class BoundingBox {
public:
    // default constructor
    BoundingBox();
    
    // initialize with specified components
    BoundingBox(const Eigen::Vector3d& min0, const Eigen::Vector3d& max0);
    
    // initialize with specified components
    BoundingBox(const Eigen::Vector3d& p);
    
    // expand bounding box to include point/ bbox
    void expandToInclude(const Eigen::Vector3d& p);
    void expandToInclude(const BoundingBox& b);
    
    // computes oriented bounding box using PCA
    void computeOrientedBox(std::vector<Eigen::Vector3d>& positions);
    
    // computes bounding box oriented along XZ axis
    void computeXZOrientedBox(std::vector<Eigen::Vector3d>& positions);
    
    // return the max dimension
    int maxDimension() const;
    
    // check if bounding box and face intersect
    bool intersect(const BoundingBox& boundingBox, double& dist) const;
    
    // check if oriented bounding boxes intersect
    bool intersectOriented(const BoundingBox& boundingBox, double& dist) const;
    
    // checks if ray intersects box and returns distance to intersection point
    bool intersect(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, double& dist) const;
    
    // checks if ray intersects oriented box and returns distance to intersection point
    bool intersectOriented(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, double& dist) const;
    
    // member variables
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    Eigen::Vector3d extent;
    
    // oriented box
    Eigen::Vector3d center;
    Eigen::Vector3d xAxis, yAxis, zAxis;
    double halfLx, halfLy, halfLz;
    
    // convex hull box
    std::vector<Eigen::Vector3d> orientedPoints;
};

#endif