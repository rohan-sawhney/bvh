#ifndef TYPES_H
#define TYPES_H

#include <stdlib.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include "math.h"
#include <Eigen/Core>
#include <Eigen/Dense>

class Vertex;
class Face;
class Mesh;
class MeshIO;
class BoundingBox;

typedef std::vector<Vertex>::iterator VertexIter;
typedef std::vector<Vertex>::const_iterator VertexCIter;
typedef std::vector<Face>::iterator FaceIter;
typedef std::vector<Face>::const_iterator FaceCIter;
typedef std::vector<Eigen::Vector3d>::iterator VectorIter;
typedef std::vector<Eigen::Vector3d>::const_iterator VectorCIter;

#endif
