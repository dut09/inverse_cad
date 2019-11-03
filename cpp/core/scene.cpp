#include "core/scene.h"
#include "core/common.h"
#include "core/file_helper.h"

Scene::Scene() {
    std::string input =
        "OFF\n\
        8 12 0\n\
        -1 -1 -1\n\
        -1 1 -1\n\
        1 1 -1\n\
        1 -1 -1\n\
        -1 -1 1\n\
        -1 1 1\n\
        1 1 1\n\
        1 -1 1\n\
        3  0 1 3\n\
        3  3 1 2\n\
        3  0 4 1\n\
        3  1 4 5\n\
        3  3 2 7\n\
        3  7 2 6\n\
        3  4 0 3\n\
        3  7 4 3\n\
        3  6 4 7\n\
        3  6 5 4\n\
        3  1 5 6\n\
        3  2 1 6";
    std::stringstream ss;
    ss << input;
    Polyhedron poly1, poly2;
    ss >> poly1;
    std::ofstream poly1_file("poly1.off");
    write_off(poly1_file, poly1);

    input =
        "OFF\n\
        8 12 0\n\
        -0.5 -0.5 -0.5\n\
        -0.5 2 -0.5\n\
        0.5 2 -0.5\n\
        0.5 -0.5 -0.5\n\
        -0.5 -0.5 0.5\n\
        -0.5 2 0.5\n\
        0.5 2 0.5\n\
        0.5 -0.5 0.5\n\
        3  0 1 3\n\
        3  3 1 2\n\
        3  0 4 1\n\
        3  1 4 5\n\
        3  3 2 7\n\
        3  7 2 6\n\
        3  4 0 3\n\
        3  7 4 3\n\
        3  6 4 7\n\
        3  6 5 4\n\
        3  1 5 6\n\
        3  2 1 6";
    std::stringstream ss2;
    ss2 << input;
    ss2 >> poly2;
    std::ofstream poly2_file("poly2.off");
    write_off(poly2_file, poly2);

    Nef_polyhedron nef1(poly1);
    Nef_polyhedron nef2(poly2);

    objects_ = nef1 - nef2;
}

void Scene::LoadScene(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    std::ifstream input(file_name);
    input >> objects_;
    CheckError(objects_.is_simple(), "The current scene is not a 2-manifold.");

    // Now construct the data structure. The code below is quite inefficient. However, since we are almost always
    // dealing with tiny scenes, I think the inefficiency is tolerable.
    target_vertices_.clear();
    const int vertex_num = static_cast<int>(objects_.number_of_vertices());
    target_vertices_.reserve(vertex_num);
    Nef_polyhedron::Vertex_const_iterator v_iter;
    for (v_iter = objects_.vertices_begin(); v_iter != objects_.vertices_end(); ++v_iter) {
        const real x = CGAL::to_double(v_iter->point().x());
        const real y = CGAL::to_double(v_iter->point().y());
        const real z = CGAL::to_double(v_iter->point().z());
        Vector3r p(x, y, z);
        target_vertices_.push_back(p);
    }

    target_half_edges_.clear();
    target_half_edge_twins_.clear();
    const int half_edge_num = static_cast<int>(objects_.number_of_halfedges());
    target_half_edges_.reserve(half_edge_num);
    target_half_edge_twins_.reserve(half_edge_num);
    Nef_polyhedron::Halfedge_const_iterator e_iter;
    for (e_iter = objects_.halfedges_begin(); e_iter != objects_.halfedges_end(); ++e_iter) {
        const real sx = CGAL::to_double(e_iter->source()->point().x());
        const real sy = CGAL::to_double(e_iter->source()->point().y());
        const real sz = CGAL::to_double(e_iter->source()->point().z());
        const int source_idx = GetVertexIndex(Vector3r(sx, sy, sz));

        const real tx = CGAL::to_double(e_iter->target()->point().x());
        const real ty = CGAL::to_double(e_iter->target()->point().y());
        const real tz = CGAL::to_double(e_iter->target()->point().z());
        const int target_idx = GetVertexIndex(Vector3r(tx, ty, tz));
        target_half_edges_.push_back(std::make_pair(source_idx, target_idx));
    }
    // Construct the twin edges.
    for (int i = 0; i < half_edge_num; ++i) {
        target_half_edge_twins_[i] = GetHalfEdgeIndex(target_half_edges_[i].second, target_half_edges_[i].first);
    }
}

void Scene::LoadTarget(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    std::ifstream input(file_name);
    input >> target_;
    CheckError(target_.is_simple(), "The target is not a 2-manifold.");
}

void Scene::ListAllVertices() {
    std::cout << "Vertex number " << target_vertices_.size() << std::endl;
    int idx = 0;
    for (const auto& v : target_vertices_) {
        const real x = v.x(), y = v.y(), z = v.z();
        std::cout << "v" << idx << "\t" << x << "\t" << y << "\t" << z << std::endl;
        ++idx;
    }
}

void Scene::ListAllEdges() {
    std::cout << "Edge number " << target_half_edges_.size() << std::endl;
    int idx = 0;
    for (const auto& e : target_half_edges_) {
        std::cout << "e" << idx << "\tv" << e.first << "\tv" << e.second << "\ttwin\t"
            << target_half_edge_twins_[idx] << std::endl;
        ++idx;
    }
}

void Scene::ListAllFaces() {
    // TODO.
}

void Scene::Extrude(const std::string& face_name, const std::vector<Vector3r>& polygon, const Vector3r& dir, const char op) {
    // TODO: use the face name to project the polygon.

    // Create the new polyhedron.
    std::stringstream ss;
    const int poly_dof = static_cast<int>(polygon.size());

    // Check the orientation of the polygon.
    std::vector<Vector3r> offset_polygon;
    for (const auto& v : polygon) offset_polygon.push_back(v - polygon[0]);
    real vol = 0;
    for (int i = 0; i < poly_dof; ++i) {
        const int next_i = (i + 1) % poly_dof;
        const Vector3r v0 = offset_polygon[i], v1 = offset_polygon[next_i];
        vol += v0.cross(v1).dot(dir);
    }
    // Do nothing if the extrusion is degenerated.
    if (vol == 0) return;

    const bool reversed = vol < 0;
    // Header.
    ss << "OFF" << std::endl
        << poly_dof * 2 << " " << poly_dof + 2 << " " << poly_dof * 3 << std::endl;
    // Vertices.
    for (const auto& v : polygon) {
        ss << v.x() << " " << v.y() << " " << v.z() << std::endl;
    }
    for (const auto& v : polygon) {
        const Vector3r v2 = v + dir;
        ss << v2.x() << " " << v2.y() << " " << v2.z() << std::endl;
    }
    // Faces.
    if (reversed) {
        ss << poly_dof;
        for (int i = 0; i < poly_dof; ++i) ss << " " << i;
        ss << std::endl << poly_dof;
        for (int i = poly_dof - 1; i >= 0; --i) ss << " " << i + poly_dof;
        ss << std::endl;

        for (int i = 0; i < poly_dof; ++i) {
            const int j = (i + 1) % poly_dof;
            ss << 4 << " " << j << " " << i << " " << i + poly_dof << " " << j + poly_dof << std::endl;
        }
    } else {
        ss << poly_dof;
        for (int i = 0; i < poly_dof; ++i) ss << " " << i + poly_dof;
        ss << std::endl << poly_dof;
        for (int i = poly_dof - 1; i >= 0; --i) ss << " " << i;
        ss << std::endl;

        for (int i = 0; i < poly_dof; ++i) {
            const int j = (i + 1) % poly_dof;
            ss << 4 << " " << i << " " << j << " " << j + poly_dof << " " << i + poly_dof << std::endl;
        }
    }

    Polyhedron poly;
    ss >> poly;
    Nef_polyhedron nef_poly(poly);
    CheckError(nef_poly.is_simple(), "The input is not a 2-manifold.");

    // Boolean operation.
    CheckError(op == '+' || op == '-', "We only support union and difference for now.");
    if (op == '+') {
        objects_ = (objects_ + nef_poly).regularization();
    } else {
        objects_ = (objects_ - nef_poly).regularization();
    }
}

void Scene::SaveScene(const std::string& file_name) {
    // Check the type of the file name.
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && (name_and_ext[1] == "off" || name_and_ext[1] == "nef3"), "Invalid file name.");
    CheckError(objects_.is_simple(), "The current scene is not a 2-manifold.");
    PrepareToCreateFile(file_name);

    const std::string ext = name_and_ext[1];
    if (ext == "off") {
        Polyhedron poly;
        objects_.convert_to_polyhedron(poly);
        std::ofstream output(file_name);
        write_off(output, poly);
    } else {
        std::ofstream output(file_name);
        output << objects_;
    }
}

void Scene::Convert(const std::string& in_file_name, const std::string& out_file_name) const {
    const std::string in_ext = GetFileExtension(in_file_name);
    const std::string out_ext = GetFileExtension(out_file_name);
    CheckError((in_ext == "off" && out_ext == "nef3") || (in_ext == "nef3" && out_ext == "off"), "Invalid input and output extensions");
    PrepareToCreateFile(out_file_name);

    if (in_ext == "off") {
        // off -> nef3.
        std::ifstream in_file(in_file_name);
        Polyhedron mesh;
        read_off(in_file, mesh);
        Nef_polyhedron poly(mesh);
        CheckError(poly.is_simple(), "The input is not a 2-manifold.");

        std::ofstream out_file(out_file_name);
        out_file << poly;
    } else {
        // nef3 -> off.
        std::ifstream in_file(in_file_name);
        Nef_polyhedron poly;
        in_file >> poly;
        CheckError(poly.is_simple(), "The input is not a 2-manifold.");

        Polyhedron mesh;
        poly.convert_to_polyhedron(mesh);
        std::ofstream out_file(out_file_name);
        write_off(out_file, mesh);
    }
}

const int Scene::GetVertexIndex(const Vector3r& vertex) const {
    int idx = 0;
    for (const auto& v : target_vertices_) {
        if (v.x() == vertex.x() && v.y() == vertex.y() && v.z() == vertex.z()) break;
        ++idx;
    }
    return idx;
}

const std::string Scene::GetVertexName(const Vector3r& vertex) const {
    return "v" + std::to_string(GetVertexIndex(vertex));
}

const int Scene::GetHalfEdgeIndex(const int source, const int target) const {
    int idx = 0;
    for (const auto& e : target_half_edges_) {
        if (e.first == source && e.second == target) break;
        ++idx;
    }
    return idx;
}

const std::string Scene::GetHalfEdgeName(const int source, const int target) const {
    return "e" + std::to_string(GetHalfEdgeIndex(source, target));
}