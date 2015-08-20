#include "ZFighter.h"
#include <random>

#define EPSILON 1e-6

bool shareEdge(const Mesh& mesh, const int fIdx1, const int fIdx2)
{
    // count number of shared vertices between the two triangles
    int sharedVerts = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (mesh.vertices[mesh.faces[fIdx1].indices[i]].index ==
                mesh.vertices[mesh.faces[fIdx2].indices[j]].index) {
                sharedVerts ++;
            }
        }
    }
    
    return sharedVerts > 1;
}

bool intersect(const Eigen::Vector3d& p, const Eigen::Vector3d& r,
               const Eigen::Vector3d& q, const Eigen::Vector3d& s)
{
    double rs = r.cross(s).norm();
    if (rs < EPSILON) {
        return false;
    }
    
    Eigen::Vector3d qp = q-p;
    double t = qp.cross(s).norm() / rs;
    double u = qp.cross(r).norm() / rs;
    
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

bool overlap(const Mesh& mesh, const int fIdx1, const int fIdx2)
{
    // pick a vertex from the first triangle
    Eigen::Vector3d t1 = mesh.vertices[mesh.faces[fIdx1].indices[1]].position;
    
    // find the normal to the face
    Eigen::Vector3d v1 = mesh.vertices[mesh.faces[fIdx1].indices[0]].position - t1;
    Eigen::Vector3d v2 = mesh.vertices[mesh.faces[fIdx1].indices[2]].position - t1;

    Eigen::Vector3d normal = v1.cross(v2);
    
    // check if sum of dot x normal from vertices of second triangle to t1 is close to zero
    Eigen::Vector3d t11 = mesh.vertices[mesh.faces[fIdx2].indices[0]].position - t1;
    Eigen::Vector3d t12 = mesh.vertices[mesh.faces[fIdx2].indices[1]].position - t1;
    Eigen::Vector3d t13 = mesh.vertices[mesh.faces[fIdx2].indices[2]].position - t1;
    
    double dot = fabs(normal.dot(t11) + normal.dot(t12) + normal.dot(t13));
    
    if (dot < EPSILON) { // triangles are coplanar
        
        // check if any of the edges of the two triangles overlap
        for (int i = 0; i < 3; i++) {
            int nextI = (i+1) % 3;
            Eigen::Vector3d l1 = mesh.vertices[mesh.faces[fIdx1].indices[i]].position -
                                 mesh.vertices[mesh.faces[fIdx1].indices[nextI]].position;
            
            for (int j = 0; j < 3; j++) {
                int nextJ = (j+1) % 3;
                Eigen::Vector3d l2 = mesh.vertices[mesh.faces[fIdx2].indices[j]].position -
                                     mesh.vertices[mesh.faces[fIdx2].indices[nextJ]].position;
        
                if (intersect(mesh.vertices[mesh.faces[fIdx1].indices[i]].position, l1,
                              mesh.vertices[mesh.faces[fIdx2].indices[j]].position, l2)) {
                    return true;
                }
            }
        }
    }

    return false;
}

void ZFighter::process(Mesh& mesh)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.009,0.012);

    for (int i = 0; i < (int)mesh.faces.size()-1; i++) {
        for (int j = i+1; j < (int)mesh.faces.size(); j++) {
            if (!shareEdge(mesh, i, j) && overlap(mesh, i, j)) {
                double r = distribution(generator);
                if (rand() / (double)RAND_MAX > 0.5) r = -r;
                
                for (int k = 0; k < 3; k++) {
                    int idx = mesh.faces[i].indices[k];
                    mesh.vertices[idx].position.x() += r;
                    mesh.vertices[idx].position.y() += r;
                    mesh.vertices[idx].position.z() += r;
                }
            }
        }
    }
}