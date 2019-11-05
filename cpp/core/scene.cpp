#include "core/scene.h"
#include "core/common.h"
#include "core/file_helper.h"

Scene::Scene() : canvas_loaded_(false), target_loaded_(false) {}

void Scene::LoadScene(const std::string& file_name) {
    CheckError(target_loaded_, "Please load the target first.");
    CheckError(!canvas_loaded_, "You are not supposed to load the canvas twice.");
    canvas_loaded_ = true;

    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    canvas_.Load(file_name);
    canvas_.Regularize(target_);
}

void Scene::LoadTarget(const std::string& file_name) {
    CheckError(!target_loaded_, "You are not supposed to load the target twice.");
    target_loaded_ = true;

    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    target_.Load(file_name);
}

void Scene::ListSceneVertices() const {
    canvas_.ListVertices();
}

void Scene::ListSceneEdges() const {
    canvas_.ListEdges();
}

void Scene::ListSceneFaces() const {
    canvas_.ListFacets();
}

void Scene::ListTargetVertices() const {
    target_.ListVertices();
}

void Scene::ListTargetEdges() const {
    target_.ListEdges();
}

void Scene::ListTargetFaces() const {
    target_.ListFacets();
}

void Scene::Extrude(const std::vector<Vector3r>& polygon, const Vector3r& dir, const char op) {
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
        canvas_ += nef_poly;
    } else {
        canvas_ -= nef_poly;
    }
    canvas_.Regularize(target_);
}

void Scene::ExtrudeFromTargetRef(const int f_idx, const int loop_idx,
    const int v_source, const int v_target, const char op) {
    Nef_polyhedron nef_poly = target_.BuildExtrusionFromRef(f_idx, loop_idx, v_source, v_target);
    CheckError(nef_poly.is_simple(), "The input is not a 2-manifold.");

    // Boolean operation.
    CheckError(op == '+' || op == '-', "We only support union and difference for now.");
    if (op == '+') {
        canvas_ += nef_poly;
    } else {
        canvas_ -= nef_poly;
    }
    canvas_.Regularize(target_);
}

void Scene::ExtrudeFromSceneRef(const int f_idx, const int loop_idx,
    const int v_source, const int v_target, const char op) {
    Nef_polyhedron nef_poly = canvas_.BuildExtrusionFromRef(f_idx, loop_idx, v_source, v_target);
    CheckError(nef_poly.is_simple(), "The input is not a 2-manifold.");

    // Boolean operation.
    CheckError(op == '+' || op == '-', "We only support union and difference for now.");
    if (op == '+') {
        canvas_ += nef_poly;
    } else {
        canvas_ -= nef_poly;
    }
    canvas_.Regularize(target_);
}

void Scene::SaveScene(const std::string& file_name) {
    canvas_.Save(file_name);
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