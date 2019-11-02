#ifndef CORE_SCENE_H
#define CORE_SCENE_H

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

class Scene {
public:
    Scene();

    void LoadScene(const std::string& file_name);
    void LoadTarget(const std::string& file_name);
    void ListInfo() const;
    void Extrude(const std::string& face_name, const std::vector<Vector3r>& polygon, const Vector3r& dir, const char op);
    void SaveScene(const std::string& file_name);
    void Convert(const std::string& in_file_name, const std::string& out_file_name) const;

private:
    Nef_polyhedron target_;
    Nef_polyhedron objects_;
};

#endif