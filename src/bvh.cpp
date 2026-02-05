#include "bvh.h"
#include <iostream>

//https://www.youtube.com/watch?v=C1H4zIiCOaI





template<typename T>
void BVH<T>::print() const
{
    if (!root_)
    {
        std::cout << "BVH is empty.\n";
        return;
    }
    
    int emptyNodes = 0;
    int heavyNodes = 0;
    
    printNode(*root_, 0, emptyNodes, heavyNodes);
    
    std::cout << "--------------------------------\n";
    std::cout << "BVH Statistics:\n";
    std::cout << "Total Empty Nodes (0 tris): " << emptyNodes << "\n";
    std::cout << "Total Heavy Nodes (>8 tris): " << heavyNodes << "\n";
    std::cout << "--------------------------------\n";
}

template<typename T>
void BVH<T>::printNode(const Node & node, int depth, int& emptyCount, int& heavyCount) const
{
    
    if (node.triangles.empty()) {
        emptyCount++;
    }
    if (node.triangles.size() > 8) {
        heavyCount++;
    }
    
    for (int i = 0; i < depth; i++)
        std::cout << "  ";

    const Vector3 size = node.box.size();
    const Vector3 center = node.box.center();

    std::cout << "Node(depth=" << depth
        << ", tris=" << node.triangles.size()
        << ", center=[" << center.x() << ", " << center.y() << ", " << center.z() << "]"
        << ", size=[" << size.x() << ", " << size.y() << ", " << size.z() << "])";

    
    if (node.triangles.size() > 8) std::cout << " <--- HEAVY";
    if (node.triangles.empty() && !node.childA && !node.childB) std::cout << " <--- USELESS LEAF";

    std::cout << "\n";

    if (node.childA)
        printNode(*node.childA, depth + 1, emptyCount, heavyCount);

    if (node.childB)
        printNode(*node.childB, depth + 1, emptyCount, heavyCount);
}
