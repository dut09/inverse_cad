#ifndef CORE_TRIANGULATION_H
#define CORE_TRIANGULATION_H

// For triangulation.
#include "core/config.h"
#include "CGAL/Exact_predicates_exact_constructions_kernel.h"
#include "CGAL/Random.h"
#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"
#include "CGAL/Constrained_Delaunay_triangulation_2.h"
#include "CGAL/Triangulation_vertex_base_with_id_2.h"
#include "CGAL/Triangulation_face_base_with_info_2.h"
#include "CGAL/Polygon_2.h"

// For triangulation.
// https://doc.cgal.org/4.14.2/Triangulation_2/index.html
struct FaceInfo2 {
    FaceInfo2() {}
    int nesting_level;
    bool visited;
    bool in_domain() { 
        return nesting_level % 2 == 1;
    }
};

typedef CGAL::Exact_predicates_exact_constructions_kernel                       Exact_kernel;
typedef Exact_kernel::Vector_3                                                  Vector_3;
typedef Exact_kernel::Point_3                                                   Point_3;
typedef CGAL::Triangulation_vertex_base_with_id_2<Exact_kernel>                 Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, Exact_kernel>      Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<Exact_kernel, Fbb>          Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                            TDS;
typedef CGAL::Exact_predicates_tag                                              Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Exact_kernel, TDS, Itag>     CDT;
typedef CGAL::Polygon_2<Exact_kernel>                                           Polygon_2;

const Vector_3 GetPolygonNormal(const std::vector<Point_3>& polygon);
CDT Triangulate(const std::vector<std::vector<Point_3>>& polygon, const std::vector<std::vector<int>>& vertex_cycle);
CDT Triangulate(const std::vector<Point_3>& polygon, const std::vector<int>& vertex_cycle);

#endif