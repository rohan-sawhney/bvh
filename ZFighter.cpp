#include "ZFighter.h"
#include "Mesh.h"
#include "Bvh.h"
#include <random>

void ZFighter::process(Mesh& mesh) const
{
    Bvh bvh(&mesh);
   
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.00009,0.00012);
    
    // if any two triangles overlap, displace one of them
    for (int i = 0; i < (int)mesh.faces.size(); i++) {
    
        Eigen::Vector3d normal = mesh.faces[i].normal(mesh);
        if (bvh.doesOverlap(i, normal) != -1) {

            double r = distribution(generator);
            if (rand() / (double)RAND_MAX > 0.5) r = -r;
            
            for (int k = 0; k < 3; k++) {
                mesh.vertices[mesh.faces[i].indices[k]].position += r * normal;
            }
        }
    }
}