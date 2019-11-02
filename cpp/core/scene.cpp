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
}

void Scene::LoadTarget(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    std::ifstream input(file_name);
    input >> target_;
    CheckError(target_.is_simple(), "The target is not a 2-manifold.");
}

void Scene::ListAllVertices() {
    const int num_vertices = static_cast<int>(objects_.number_of_vertices());
    std::cout << "Vertex number " << num_vertices << std::endl;
    int idx = 0;
    Nef_polyhedron::Vertex_const_iterator iter; 
    for (iter = objects_.vertices_begin(); iter != objects_.vertices_end(); ++iter) {
        const auto& point = iter->point();
        // We can use std::cout << point to print its value directly but the code below shows better how
        // to access data members of a point.
        const real x = CGAL::to_double(point.x());
        const real y = CGAL::to_double(point.y());
        const real z = CGAL::to_double(point.z());
        std::cout << "v" << idx << "\t" << x << "\t" << y << "\t" << z << std::endl;
        ++idx;
    }
}

void Scene::ListAllEdges() {
    // TODO.
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