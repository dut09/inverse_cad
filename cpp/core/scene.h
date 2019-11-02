#ifndef CORE_SCENE_H
#define CORE_SCENE_H

#include "core/config.h"

class Scene {
public:
    Scene();

    void LoadScene(const std::string& file_name);
    void SaveScene(const std::string& file_name);

private:
};

#endif