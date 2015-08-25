#include "Face.h"
#include "Mesh.h"
#include "BoundingBox.h"
#define EPSILON 1e-6

Face::Face()
{
    
}

Face::Face(const Eigen::Vector3d& indices0):
indices(indices0)
{
    
}

Eigen::Vector3d Face::normal(const Mesh& mesh) const
{
    Eigen::Vector3d p1 = mesh.vertices[indices[0]].position;
    Eigen::Vector3d p2 = mesh.vertices[indices[1]].position;
    Eigen::Vector3d p3 = mesh.vertices[indices[2]].position;
    
    // find the normal to the face
    Eigen::Vector3d v1 = p1 - p2;
    Eigen::Vector3d v2 = p3 - p2;
    
    Eigen::Vector3d normal = v1.cross(v2);
    normal.normalize();
    
    return normal;
}

BoundingBox Face::boundingBox(const Mesh& mesh) const
{
    // assumes face is a triangle
    Eigen::Vector3d p1 = mesh.vertices[indices[0]].position;
    Eigen::Vector3d p2 = mesh.vertices[indices[1]].position;
    Eigen::Vector3d p3 = mesh.vertices[indices[2]].position;
    
    Eigen::Vector3d min = p1;
    Eigen::Vector3d max = p1;
    
    if (p2.x() < min.x()) min.x() = p2.x();
    if (p3.x() < min.x()) min.x() = p3.x();
    
    if (p2.y() < min.y()) min.y() = p2.y();
    if (p3.y() < min.y()) min.y() = p3.y();
    
    if (p2.z() < min.z()) min.z() = p2.z();
    if (p3.z() < min.z()) min.z() = p3.z();
    
    if (p2.x() > max.x()) max.x() = p2.x();
    if (p3.x() > max.x()) max.x() = p3.x();
    
    if (p2.y() > max.y()) max.y() = p2.y();
    if (p3.y() > max.y()) max.y() = p3.y();
    
    if (p2.z() > max.z()) max.z() = p2.z();
    if (p3.z() > max.z()) max.z() = p3.z();
    
    return BoundingBox(min, max);
}

Eigen::Vector3d Face::centroid(const Mesh& mesh) const
{
    // assumes face is a triangle
    Eigen::Vector3d centroid = (mesh.vertices[indices[0]].position +
                                mesh.vertices[indices[1]].position +
                                mesh.vertices[indices[2]].position) / 3;
    return centroid;
}

bool Face::shareEdge(const Mesh& mesh, const int fIdx) const
{
    // count number of shared vertices between the two triangles
    int sharedVerts = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (mesh.vertices[indices[i]].index ==
                mesh.vertices[mesh.faces[fIdx].indices[j]].index) {
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

bool Face::overlap(const Mesh& mesh, const int fIdx, const Eigen::Vector3d& normal) const
{
    // pick a vertex from the first triangle
    Eigen::Vector3d t1 = mesh.vertices[indices[1]].position;
    
    // check if sum of dot x normal from vertices of second triangle to t1 is close to zero
    Eigen::Vector3d t11 = mesh.vertices[mesh.faces[fIdx].indices[0]].position - t1;
    Eigen::Vector3d t12 = mesh.vertices[mesh.faces[fIdx].indices[1]].position - t1;
    Eigen::Vector3d t13 = mesh.vertices[mesh.faces[fIdx].indices[2]].position - t1;
    
    double dot = fabs(normal.dot(t11) + normal.dot(t12) + normal.dot(t13));
    
    if (dot < EPSILON) { // triangles are coplanar
        
        // check if any of the edges of the two triangles overlap
        for (int i = 0; i < 3; i++) {
            int nextI = (i+1) % 3;
            Eigen::Vector3d l1 = mesh.vertices[indices[i]].position -
                                 mesh.vertices[indices[nextI]].position;
            
            for (int j = 0; j < 3; j++) {
                int nextJ = (j+1) % 3;
                Eigen::Vector3d l2 = mesh.vertices[mesh.faces[fIdx].indices[j]].position -
                                     mesh.vertices[mesh.faces[fIdx].indices[nextJ]].position;
                
                if (intersect(mesh.vertices[indices[i]].position, l1,
                              mesh.vertices[mesh.faces[fIdx].indices[j]].position, l2)) {
                    return true;
                }
            }
        }
    }
    
    return false;
}
