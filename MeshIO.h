#ifndef MESH_IO_H
#define MESH_IO_H

#include <fstream>
#include "Types.h"

class MeshData;

class MeshIO {
public:
    // reads data from obj file
    static bool read(std::ifstream& in, Mesh& mesh);
    
    // writes data in obj format
    static void write(std::ofstream& out, const Mesh& mesh);
};

#endif
