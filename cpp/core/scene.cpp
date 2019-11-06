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

void Scene::ExtrudeFromString(const std::string& str) {
    const std::vector<std::string> words = SplitString(str);
    const int word_num = static_cast<int>(words.size());
    std::istringstream iss(str);
    std::string name;
    iss >> name;
    // extrude <x y z> <x y z> ... <x y z> <dx, dy dz> <+|->.
    const int poly_dof = static_cast<int>((word_num - 5) / 3);
    std::vector<Exact_kernel::Point_3> polygon(poly_dof);
    for (int i = 0; i < poly_dof; ++i) {
        iss >> polygon[i];
    }
    Exact_kernel::Vector_3 dir;
    iss >> dir;
    Nef_polyhedron nef_poly = canvas_.BuildExtrusionFromData(polygon, Aff_transformation_3(CGAL::TRANSLATION, dir));
    CheckError(nef_poly.is_simple(), "The input is not a 2-manifold.");

    // Boolean operation.
    CheckError(words[word_num - 1].size() == 1u, "The last input has to be a char.");
    const char op = words[word_num - 1][0];
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