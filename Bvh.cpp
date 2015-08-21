#include "Bvh.h"
#include "BoundingBox.h"
#include "Mesh.h"
#include <stack>

class Node {
public:
    BoundingBox boundingBox;
    
    int startId, range, rightOffset;
};

Bvh::Bvh(Mesh *meshPtr0):
meshPtr(meshPtr0)
{
    
}

struct BvhNodeEntry {
    // parent id
    int parentId;
    
    // range of objects covered by the node
    int startId, endId;
};

BoundingBox Bvh::computeBoundingBox(const int fId)
{
    // assumes face is a triangle
    Eigen::Vector3d p1 = meshPtr->vertices[meshPtr->faces[fId].indices[0]].position;
    Eigen::Vector3d p2 = meshPtr->vertices[meshPtr->faces[fId].indices[1]].position;
    Eigen::Vector3d p3 = meshPtr->vertices[meshPtr->faces[fId].indices[2]].position;
    
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

Eigen::Vector3d Bvh::computeCentroid(const int fId)
{
    // assumes face is a triangle
    Eigen::Vector3d centroid = (meshPtr->vertices[meshPtr->faces[fId].indices[0]].position +
                                meshPtr->vertices[meshPtr->faces[fId].indices[1]].position +
                                meshPtr->vertices[meshPtr->faces[fId].indices[2]].position) / 3;
    return centroid;
}

void Bvh::build()
{
    std::stack<BvhNodeEntry> stack;
    const int faceCount = (int)meshPtr->faces.size();
    
    BvhNodeEntry nodeEntry;
    nodeEntry.parentId = -1;
    nodeEntry.startId = 0;
    nodeEntry.endId = faceCount;
    stack.push(nodeEntry);
    
    std::vector<Node> nodes;
    nodes.reserve(faceCount * 2);
    
    while (!stack.empty()) {
        // pop item off the stack and create a node
        nodeEntry = stack.top();
        stack.pop();
        int startId = nodeEntry.startId;
        int endId = nodeEntry.endId;
        
        nodeCount ++;
        Node node;
        node.startId = startId;
        node.range = endId - startId;
        node.rightOffset = 0; // TODO
        
        // calculate bounding box
        BoundingBox boundingBox(computeBoundingBox(startId));
        BoundingBox boundingCentroid(computeCentroid(startId));
        for (int i = 0; i < endId; i++) {
            boundingBox.expandToInclude(computeBoundingBox(i));
            boundingCentroid.expandToInclude(computeCentroid(i));
        }
        node.boundingBox = boundingBox;
        
        // if node is a leaf
        if (node.range <= 1) {
            node.rightOffset = 0;
            leafCount ++;
        }
        
        nodes.push_back(node);
        
        if (nodeEntry.parentId != -1) {
            nodes[nodeEntry.parentId].rightOffset --;
            
            if (nodes[nodeEntry.parentId].rightOffset == 5) { // TODO
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
            if (computeCentroid(i)[maxDimension] < splitCoord) {
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

bool Bvh::doesOverlap(const Face& f)
{
    return false;
}