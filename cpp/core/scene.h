#ifndef CORE_SCENE_H
#define CORE_SCENE_H

#include "core/nef3_wrapper.h"
#include "core/python_struct.h"

class Scene {
public:
    Scene();

    const Scene Clone() const {
        return Scene(*this);
    }
    void SetTargetFromOtherScene(const Scene& other);

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

    // For Python binding.
    const int GetSceneVertexNumber() const { return canvas_.GetVertexNumber(); }
    const int GetTargetVertexNumber() const { return target_.GetVertexNumber(); }
    const VertexInfo GetSceneVertex(const int vid) const { return canvas_.GetVertexInfo(vid); }
    const VertexInfo GetTargetVertex(const int vid) const { return target_.GetVertexInfo(vid); }

    const int GetSceneHalfEdgeNumber() const { return canvas_.GetHalfEdgeNumber(); }
    const int GetTargetHalfEdgeNumber() const { return target_.GetHalfEdgeNumber(); }
    const HalfEdgeInfo GetSceneHalfEdge(const int eid) const { return canvas_.GetHalfEdgeInfo(eid); }
    const HalfEdgeInfo GetTargetHalfEdge(const int eid) const { return target_.GetHalfEdgeInfo(eid); }

    const int GetSceneHalfFacetNumber() const { return canvas_.GetHalfFacetNumber(); }
    const int GetTargetHalfFacetNumber() const { return target_.GetHalfFacetNumber(); }
    const HalfFacetInfo GetSceneHalfFacet(const int eid) const { return canvas_.GetHalfFacetInfo(eid); }
    const HalfFacetInfo GetTargetHalfFacet(const int eid) const { return target_.GetHalfFacetInfo(eid); }

private:
    Nef3Wrapper target_;
    Nef3Wrapper canvas_;
};

#endif