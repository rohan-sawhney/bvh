#include "Bvh.h"
#include "Mesh.h"
#include <stack>

Bvh::Bvh(Mesh *meshPtr0, int leafSize0):
meshPtr(meshPtr0),
nodeCount(0),
leafCount(0),
leafSize(leafSize0)
{
    build();
}

struct NodeEntry {
    // parent id
    int parentId;
    
    // range of objects covered by the node
    int startId, endId;
};

void Bvh::build()
{
    std::stack<NodeEntry> stack;
    const int faceCount = (int)meshPtr->faces.size();
    
    NodeEntry nodeEntry;
    nodeEntry.parentId = -1;
    nodeEntry.startId = 0;
    nodeEntry.endId = faceCount;
    stack.push(nodeEntry);
    
    Node node;
    Face f;
    std::vector<Node> nodes;
    nodes.reserve(faceCount * 2);
    
    while (!stack.empty()) {
        // pop item off the stack and create a node
        nodeEntry = stack.top();
        stack.pop();
        int startId = nodeEntry.startId;
        int endId = nodeEntry.endId;

        nodeCount ++;
        node.startId = startId;
        node.range = endId - startId;
        node.rightOffset = 2;
        
        // calculate bounding box
        f = meshPtr->faces[startId];
        BoundingBox boundingBox(f.boundingBox(*meshPtr));
        BoundingBox boundingCentroid(f.centroid(*meshPtr));
        for (int i = startId+1; i < endId; i++) {
            f = meshPtr->faces[i];
            boundingBox.expandToInclude(f.boundingBox(*meshPtr));
            boundingCentroid.expandToInclude(f.centroid(*meshPtr));
        }
        node.boundingBox = boundingBox;

        // if node is a leaf
        if (node.range <= leafSize) {
            node.rightOffset = 0;
            leafCount ++;
        }
        
        nodes.push_back(node);
        
        // compute parent's rightOffset
        if (nodeEntry.parentId != -1) {
            nodes[nodeEntry.parentId].rightOffset --;
            
            if (nodes[nodeEntry.parentId].rightOffset == 0) {
                nodes[nodeEntry.parentId].rightOffset = nodeCount - 1 - nodeEntry.parentId;
            }
        }
        
        // if a leaf, no need to subdivide
        if (node.rightOffset == 0) {
            continue;
        }
        
        // find the center of the longest dimension
        int maxDimension = boundingCentroid.maxDimension();
        double splitCoord = 0.5 * (boundingCentroid.min[maxDimension] +
                                   boundingCentroid.max[maxDimension]);
        
        // partition faces
        int mid = startId;
        for (int i = startId; i < endId; i++) {
            f = meshPtr->faces[i];
            if (f.centroid(*meshPtr)[maxDimension] < splitCoord) {
                std::swap(meshPtr->faces[i], meshPtr->faces[mid]);
                mid ++;
            }
        }
        
        // in case of a bad split
        if (mid == startId || mid == endId) {
            mid = startId + (endId-startId) / 2;
        }
        
        // push right child
        nodeEntry.startId = mid;
        nodeEntry.endId = endId;
        nodeEntry.parentId = nodeCount - 1;
        stack.push(nodeEntry);
        
        // push left child
        nodeEntry.startId = startId;
        nodeEntry.endId = mid;
        nodeEntry.parentId = nodeCount - 1;
        stack.push(nodeEntry);
    }
    
    // copy node data into temp array
    for (int i = 0; i < nodeCount; i ++) {
        flatTree.push_back(nodes[i]);
    }
}

int Bvh::doesOverlap(const int fid, Eigen::Vector3d& normal) const
{
    Face f = meshPtr->faces[fid];
    BoundingBox boundingBox = f.boundingBox(*meshPtr);
    
    int id = 0;
    int closer, further;
    double dist1 = 0.0;
    double dist2 = 0.0;
    
    std::stack<int> stack;
    stack.push(id);
    
    while (!stack.empty()) {
        id = stack.top();
        stack.pop();
        
        const Node &node(flatTree[id]);
        // node is a leaf
<<<<<<< HEAD
        if (node.rightOffset == 0) {
=======
        //std::cout << "1: " << node.rightOffset << std::endl;
        if (node.rightOffset == 0) {
            //std::cout << "leaf r: " << node.range << std::endl;
>>>>>>> origin/master
            for (int i = 0; i < node.range; i++) {
                // check for overlap
                if (!f.shareEdge(*meshPtr, node.startId+i) &&
                     f.overlap(*meshPtr, node.startId+i, normal)) {
                    
                    return node.startId+i;
                }
            }
            
        } else { // not a leaf
            bool hit0 = flatTree[id+1].boundingBox.contains(boundingBox, dist1);
            bool hit1 = flatTree[id+node.rightOffset].boundingBox.contains(boundingBox, dist2);
            
<<<<<<< HEAD
=======
            //std::cout << "hit0: " << hit0 << " hit1: " << hit1 << std::endl;
>>>>>>> origin/master
            // hit both bounding boxes
            if (hit0 && hit1) {
                closer = id+1;
                further = id+node.rightOffset;
                
                if (dist2 < dist1) {
                    std::swap(dist1, dist2);
                    std::swap(closer, further);
                }
                
                // push farther node first
                stack.push(further);
                stack.push(closer);
                
            } else if (hit0) {
                stack.push(id+1);
                
            } else if (hit1) {
                stack.push(id+node.rightOffset);
            }
        }
    }
    
<<<<<<< HEAD
=======
    //std::cout << "return -1" << std::endl;
>>>>>>> origin/master
    return -1;
}