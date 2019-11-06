#ifndef CORE_SCENE_H
#define CORE_SCENE_H

#include "core/nef3_wrapper.h"

class Scene {
public:
    Scene();

    void LoadScene(const std::string& file_name);
    void LoadTarget(const std::string& file_name);
    void ListSceneVertices() const;
    void ListSceneEdges() const;
    void ListSceneFaces() const;
    void ListTargetVertices() const;
    void ListTargetEdges() const;
    void ListTargetFaces() const;
    void ExtrudeFromString(const std::string& str);
    void ExtrudeFromSceneRef(const int f_idx, const int loop_idx,
        const int v_source, const int v_target, const char op);
    void ExtrudeFromTargetRef(const int f_idx, const int loop_idx,
        const int v_source, const int v_target, const char op);
    void SaveScene(const std::string& file_name);
    void Convert(const std::string& in_file_name, const std::string& out_file_name) const;

private:
    bool canvas_loaded_;
    bool target_loaded_;

    Nef3Wrapper target_;
    Nef3Wrapper canvas_;
};

#endif