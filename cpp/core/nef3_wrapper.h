#ifndef CORE_NEF3_WRAPPER_H
#define CORE_NEF3_WRAPPER_H

#include "core/config.h"
#include "CGAL/Exact_predicates_exact_constructions_kernel.h"
#include "CGAL/Polyhedron_3.h"
#include "CGAL/Polyhedron_incremental_builder_3.h"
#include "CGAL/Nef_polyhedron_3.h"
#include "CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h"
#include "CGAL/IO/Nef_polyhedron_iostream_3.h"
#include "core/common.h"
#include "core/python_struct.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel Exact_kernel;
typedef CGAL::Polyhedron_3<Exact_kernel> Polyhedron;
typedef CGAL::Nef_polyhedron_3<Exact_kernel> Nef_polyhedron;
typedef Exact_kernel::Vector_3 Vector_3;
typedef Exact_kernel::Point_3 Point_3;
typedef Exact_kernel::Aff_transformation_3 Aff_transformation_3;
typedef Polyhedron::HalfedgeDS HalfedgeDS;

class Nef3Wrapper {
public:
    void Load(const std::string& file_name);

    void Save(const std::string& file_name);
    void Regularize(const Nef3Wrapper& other, const real eps = ToReal(1e-6));

    void operator+=(const Nef_polyhedron& other);
    void operator-=(const Nef_polyhedron& other);

    const Nef_polyhedron& poly() const { return poly_; }
    const std::vector<Point_3>& vertices() const { return vertices_; }
    const std::vector<std::pair<int, int>>& half_edges() const { return half_edges_; }
    const std::vector<int>& half_edge_twins() const { return half_edge_twins_; }
    const std::vector<std::vector<std::vector<int>>>& half_facets() const { return half_facets_; }
    const std::vector<int>& half_facet_twins() const { return half_facet_twins_; }

    const int GetVertexNumber() const { return static_cast<int>(vertices_.size()); }
    const int GetHalfEdgeNumber() const { return static_cast<int>(half_edges_.size()); }
    const int GetHalfFacetNumber() const { return static_cast<int>(half_facets_.size()); }

    void ListVertices() const;
    void ListEdges() const;
    void ListFacets() const;

    const Nef_polyhedron BuildExtrusionFromData(const std::vector<Point_3>& polygon,
        const Aff_transformation_3& dir) const;
    const Nef_polyhedron BuildExtrusionFromRef(const int f_idx, const int loop_idx,
        const int v_source, const int v_target) const;

    static const Vector3r ToEigenVector3r(const Point_3& point);
    static const Vector3r ToEigenVector3r(const Vector_3& vector);

    // For python binding.
    const VertexInfo GetVertexInfo(const int vid) const { return {
        "v" + std::to_string(vid),
        CGAL::to_double(vertices_[vid].x()),
        CGAL::to_double(vertices_[vid].y()),
        CGAL::to_double(vertices_[vid].z())
    }; }
    const HalfEdgeInfo GetHalfEdgeInfo(const int eid) const { return {
        "e" + std::to_string(eid),
        half_edges_[eid].first,
        half_edges_[eid].second,
        half_edge_twins_[eid]
    }; }
    const HalfFacetInfo GetHalfFacetInfo(const int fid) const { return {
        "f" + std::to_string(fid),
        half_facets_[fid],
        half_facet_twins_[fid],
        half_facet_outwards_[fid],
    }; }

private:
    const int GetVertexIndex(const Point_3& vertex) const;
    const int GetHalfEdgeIndex(const int source, const int target) const;

    void SyncDataStructure();
    // Given vertices, edges, and facets, compute half_facet_outwards_;
    void ComputeFacetOrientation();
    const bool IsOutwardHalfFacet(const int fid, const int vc_idx) const;

    Nef_polyhedron poly_;

    std::vector<Point_3> vertices_;
    std::vector<std::pair<int, int>> half_edges_;
    std::vector<int> half_edge_twins_;
    std::vector<std::vector<std::vector<int>>> half_facets_;
    std::vector<int> half_facet_twins_;
    std::vector<bool> half_facet_outwards_;

    std::vector<bool> vertices_match_target_;
    std::vector<bool> half_edges_match_target_;
    std::vector<bool> half_facets_match_target_;
};

#endif