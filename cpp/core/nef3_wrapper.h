#ifndef CORE_NEF3_WRAPPER_H
#define CORE_NEF3_WRAPPER_H

#include "core/config.h"
#include "CGAL/Exact_predicates_exact_constructions_kernel.h"
#include "CGAL/Polyhedron_3.h"
#include "CGAL/Surface_mesh.h"
#include "CGAL/Nef_polyhedron_3.h"
#include "CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h"
#include "CGAL/IO/Nef_polyhedron_iostream_3.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel Exact_kernel;
typedef CGAL::Polyhedron_3<Exact_kernel> Polyhedron;
typedef CGAL::Surface_mesh<Exact_kernel::Point_3> Surface_mesh;
typedef CGAL::Nef_polyhedron_3<Exact_kernel> Nef_polyhedron;

struct Nef3Wrapper {
public:
    void Load(const std::string& file_name);

    void Save(const std::string& file_name);
    void Regularize(const Nef3Wrapper& other);

    void operator+=(const Nef_polyhedron& other);
    void operator-=(const Nef_polyhedron& other);

    const Nef_polyhedron& poly() const { return poly_; }
    const std::vector<Vector3r>& vertices() const { return vertices_; }
    const std::vector<std::pair<int, int>>& half_edges() const { return half_edges_; }
    const std::vector<int>& half_edge_twins() const { return half_edge_twins_; }
    const std::vector<std::vector<std::vector<int>>>& half_facets() const { return half_facets_; }
    const std::vector<int>& half_facet_twins() const { return half_facet_twins_; }

    const int GetVertexNum() const { return static_cast<int>(vertices_.size()); }
    const int GetHalfEdgeNumber() const { return static_cast<int>(half_edges_.size()); }
    const int GetHalfFacetNumber() const { return static_cast<int>(half_facets_.size()); }

    void ListVertices() const;
    void ListEdges() const;
    void ListFacets() const;

private:
    const int GetVertexIndex(const Vector3r& vertex) const;
    const int GetHalfEdgeIndex(const int source, const int target) const;

    void SyncDataStructure();

    Nef_polyhedron poly_;

    std::vector<Vector3r> vertices_;
    std::vector<std::pair<int, int>> half_edges_;
    std::vector<int> half_edge_twins_;
    std::vector<std::vector<std::vector<int>>> half_facets_;
    std::vector<int> half_facet_twins_;
};

#endif