#ifndef VERTEX_H
#define VERTEX_H

#include "Types.h"

class Vertex {
public:
    
    // initialize with specified components
    Vertex(const Eigen::Vector3d& position0, int index0);
    
    // location in 3d
    Eigen::Vector3d position;
    
    // id between 0 and |V|-1
    int index;
};

#endif
