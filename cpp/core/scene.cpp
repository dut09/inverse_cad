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
    // TODO.
}

void Scene::SaveScene(const std::string& file_name) {
    // Check the type of the file name.
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && (name_and_ext[1] == "off" || name_and_ext[1] == "nef3"), "Invalid file name.");
    CheckError(objects_.is_simple(), "The current scene is not a 2-d manifold.");
    PrepareToCreateFile(file_name);

    const std::string ext = name_and_ext[1];
    if (ext == "off") {
        Polyhedron poly;
        objects_.convert_to_polyhedron(poly);
        std::ofstream output(file_name);
        write_off(output, poly);
    } else {
    }
}