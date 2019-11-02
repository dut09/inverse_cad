#include "core/scene.h"
#include "core/common.h"
#include "core/file_helper.h"

Scene::Scene() {
    // Create a cube.
}

void Scene::LoadScene(const std::string& file_name) {
    // TODO.
}

void Scene::SaveScene(const std::string& file_name) {
    // Check the type of the file name.
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && (name_and_ext[1] == "off" || name_and_ext[1] == "nef3"), "Invalid file name.");
    PrepareToCreateFile(file_name);

    const std::string ext = name_and_ext[1];
    if (ext == ".off") {
    } else {
    }
}