#include "ZFighter.h"
#include "Mesh.h"
#include "Bvh.h"
#include <random>

void ZFighter::process(Mesh& mesh) const
{
    Bvh bvh(&mesh);
    
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.009,0.012);
    
    Eigen::Vector3d normal;

    // if any two triangles overlap, displace one of them
    for (int i = 0; i < (int)mesh.faces.size(); i++) {
        
        if (bvh.doesOverlap(i, normal) != -1) {
            double r = distribution(generator);
            if (rand() / (double)RAND_MAX > 0.5) r = -r;
            
            for (int k = 0; k < 3; k++) {
                mesh.vertices[mesh.faces[i].indices[k]].position += r * normal;
            }
        }
        
     /*
        Face f = mesh.faces[i];
        for (int j = 0; j < (int)mesh.faces.size(); j++) {
            if (!f.shareEdge(mesh, j) && f.overlap(mesh, j, normal)) {
                double r = distribution(generator);
                if (rand() / (double)RAND_MAX > 0.5) r = -r;
                
                for (int k = 0; k < 3; k++) {
                    mesh.vertices[mesh.faces[i].indices[k]].position += r * normal;
                }
            }
        }
      */
    }
}