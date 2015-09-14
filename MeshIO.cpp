#include "MeshIO.h"
#include "Mesh.h"

class Index {
public:
    Index() {}
    
    Index(int v, int vt, int vn): position(v), uv(vt), normal(vn) {}
    
    bool operator<(const Index& i) const {
        if (position < i.position) return true;
        if (position > i.position) return false;
        if (uv < i.uv) return true;
        if (uv > i.uv) return false;
        if (normal < i.normal) return true;
        if (normal > i.normal) return false;
        
        return false;
    }
    
    int position;
    int uv;
    int normal;
};

Index parseFaceIndex(const std::string& token)
{
    std::stringstream in(token);
    std::string indexString;
    int indices[3] = {-1, -1, -1};
    
    int i = 0;
    while(getline(in,indexString,'/')) {
        if (indexString != "\\") {
            std::stringstream ss(indexString);
            ss >> indices[i++];
            
        } 
    }
    
    // decrement since indices in OBJ files are 1-based
    return Index(indices[0]-1,
                 indices[1]-1,
                 indices[2]-1);
}

bool MeshIO::read(std::ifstream& in, Mesh& mesh)
{
    mesh.vertices.clear();
    mesh.uvs.clear();
    mesh.normals.clear();
    mesh.faces.clear();
    
    std::vector<Vertex> vertices;
    std::vector<Eigen::Vector3d> uvs;
    std::vector<Eigen::Vector3d> normals;
    
    // parse obj format
    std::string line;
    while(getline(in, line)) {
        std::stringstream ss(line);
        std::string token;
        
        ss >> token;
        if (token == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            
            vertices.push_back(Vertex(Eigen::Vector3d(x, y, z), (int)vertices.size()));
            
        } else if (token == "vt") {
            double u, v;
            ss >> u >> v;
            
            uvs.push_back(Eigen::Vector3d(u, v, 0));
            
        } else if (token == "vn") {
            double x, y, z;
            ss >> x >> y >> z;
            
            normals.push_back(Eigen::Vector3d(x, y, z));
            
        } else if (token == "f") {
            Eigen::Vector3d indices;
            
            int i = 0;
            while (i < 3 && ss >> token) {
                Index index = parseFaceIndex(token);
                if (index.position < 0) {
                    getline(in, line);
                    size_t i = line.find_first_not_of("\t\n\v\f\r ");
                    index = parseFaceIndex(line.substr(i));
                }
                
                indices[i] = (int)mesh.vertices.size();
                
                mesh.vertices.push_back(vertices[index.position]);
                if (index.uv > -1) {
                    mesh.uvs.push_back(uvs[index.uv]);
                }
                
                if (index.normal > -1) {
                    mesh.normals.push_back(normals[index.normal]);
                }
                
                i ++;
            }
            
            mesh.faces.push_back(Face(indices));
        }
    }
    
    return true;
}

void MeshIO::write(std::ofstream& out, const Mesh& mesh)
{
    // write vertices
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        out << "v " << v->position.x() << " "
                    << v->position.y() << " "
                    << v->position.z() << std::endl;
    }
    
    // write uvs
    for (VectorCIter uv = mesh.uvs.begin(); uv != mesh.uvs.end(); uv++) {
        out << "vt " << uv->x() << " "
                     << uv->y() << std::endl;
    }
    
    // write normals
    for (VectorCIter n = mesh.uvs.begin(); n != mesh.normals.end(); n++) {
        out << "vn " << n->x() << " "
                     << n->y() << " "
                     << n->z() << std::endl;
    }
    
    // write faces
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        
        out << "f ";
        for (int i = 0; i < f->indices.size(); i++) {
            out << f->indices[i] << "/"
                << f->indices[i] << "/"
                << f->indices[i] << " ";
        }
        out << std::endl;
    }
}