#ifndef FACE_H
#define FACE_H

#include "Types.h"

class Face {
public:
    // initialize with specified compoenents
    Face(const Eigen::Vector3d& indices0);
    
    // indices
    Eigen::Vector3d indices;
};

#endif
