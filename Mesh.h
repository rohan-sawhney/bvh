#ifndef MESH_H
#define MESH_H

#include "Types.h"
#include "Face.h"
#include "Vertex.h"

class Mesh {
public:
    // default constructor
    Mesh();
    
    // copy constructor
    Mesh(const Mesh& mesh);
        
    // read mesh from file
    bool read(const std::string& fileName);
    
    // write mesh to file
    bool write(const std::string& fileName) const;
    
    // member variables
    std::vector<Vertex> vertices;
    std::vector<Eigen::Vector3d> uvs;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Face> faces;

private:
    // center mesh about origin and rescale to unit radius
    void normalize();
};

#endif