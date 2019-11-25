#ifndef CORE_PYTHON_STRUCT_H
#define CORE_PYTHON_STRUCT_H

#include "core/config.h"

struct VertexInfo {
    std::string name;
    real x, y, z;
};

struct HalfEdgeInfo {
    std::string name;
    int source;
    int target;
    int twin;
};

struct HalfFacetInfo {
    std::string name;
    std::vector<std::vector<int>> cycles;
    int twin;
    bool outward;
};

#endif