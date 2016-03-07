#include <assert.h>
#include <vector>
#include <iostream>
#include "QuarterEdge.hpp"

int main() {
    std::cout << "size of Edge " << sizeof(Edge) << std::endl;
    for(int i = 0; i < 8; i++)
        for(int j = 0; j < 3; j++)
            std::cout << i << " -> " << (i ^ (1 << j)) << " -> " << QuarterEdgeMesh::nextEdgeInHexahedron(i, i ^ (1 << j)) << std::endl;



}
