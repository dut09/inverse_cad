#include "core/scene.h"
// For triangulation.
#include "CGAL/Random.h"
#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"
#include "CGAL/Constrained_Delaunay_triangulation_2.h"
#include "CGAL/Triangulation_vertex_base_with_id_2.h"
#include "CGAL/Triangulation_face_base_with_info_2.h"
#include "CGAL/Polygon_2.h"

#include "core/common.h"
#include "core/file_helper.h"

// For triangulation.
// https://doc.cgal.org/4.14.2/Triangulation_2/index.html
struct FaceInfo2 {
    FaceInfo2() {}
    int nesting_level;
    bool visited;
    bool in_domain() { 
        return nesting_level % 2 == 1;
    }
};

typedef CGAL::Triangulation_vertex_base_with_id_2<Exact_kernel>                 Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, Exact_kernel>      Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<Exact_kernel, Fbb>          Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                            TDS;
typedef CGAL::Exact_predicates_tag                                              Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Exact_kernel, TDS, Itag>     CDT;
typedef CGAL::Polygon_2<Exact_kernel>                                           Polygon_2;

static void MarkDomains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border) {
    if (start->info().nesting_level != -1) return;
    std::list<CDT::Face_handle> queue;
    queue.push_back(start);
    while (!queue.empty()) {
        CDT::Face_handle fh = queue.front();
        queue.pop_front();
        if (fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for (int i = 0; i < 3; i++) {
                CDT::Edge e(fh, i);
                CDT::Face_handle n = fh->neighbor(i);
                if (n->info().nesting_level == -1) {
                    if (ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}

static void MarkDomains(CDT& cdt) {
    for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
        it->info().nesting_level = -1;
    }
    std::list<CDT::Edge> border;
    MarkDomains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty()) {
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1) {
            MarkDomains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}
// End of triangulation.

Scene::Scene() {}

void Scene::SetTargetFromOtherScene(const Scene& other) {
    target_ = other.target_;
    canvas_.Regularize(target_);
}

void Scene::LoadScene(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    canvas_.Load(file_name);
    canvas_.Regularize(target_);
}

void Scene::LoadTarget(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    target_.Load(file_name);
    canvas_.Regularize(target_);
}

const std::string Scene::GenerateRandomPolygon(const int f_idx, const bool use_target) const {
    const Nef3Wrapper& polyhedron = use_target ? target_ : canvas_;
    CheckError(0 <= f_idx && f_idx < static_cast<int>(polyhedron.half_facets().size()), "f_idx out of range.");
    CheckError(polyhedron.half_facets()[f_idx].size() == 1u, "Polygon with holes is not supported.");
    const std::vector<int>& vertex_cycle = polyhedron.half_facets()[f_idx][0];

    // Project the facet to a 2d polygon.
    // Build the local frame.
    const Point_3 v0 = polyhedron.vertices()[vertex_cycle[0]];
    const Point_3 v1 = polyhedron.vertices()[vertex_cycle[1]];
    const Point_3 v2 = polyhedron.vertices()[vertex_cycle[2]];
    Vector_3 x = v1 - v0;
    const Vector_3 z = CGAL::cross_product(x, v2 - v1);
    Vector_3 y = CGAL::cross_product(x, z);
    x /= std::sqrt(CGAL::to_double(x.squared_length()));
    y /= std::sqrt(CGAL::to_double(y.squared_length()));

    // Add points and constraints.
    CDT cdt;
    std::vector<CDT::Point> points;
    for (const int vid : vertex_cycle) {
        const Point_3 v = polyhedron.vertices()[vid];
        const CDT::Point p((v - v0) * x, (v - v0) * y);
        points.push_back(p);
        cdt.insert(p)->id() = vid;
    }
    cdt.insert_constraint(points.begin(), points.end(), true);

    // Mark facets that are inside the domain bounded by the polygon.
    MarkDomains(cdt);

    // Random generation.
    int face_cnt = 0;
    const int expected_face_cnt = static_cast<int>(points.size()) - 2;
    CGAL::Random rand;
    const int root_cnt = rand.get_int(0, expected_face_cnt);
    CDT::Face_handle root;
    for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
        if (fit->info().in_domain()) {
            fit->info().visited = false;
            if (face_cnt == root_cnt) root = CDT::Face_handle(fit);
            ++face_cnt;
        }
    }
    CheckError(face_cnt == expected_face_cnt, "Triangle number and vertex number do not match.");

    // Random BFS.
    const int target_face_cnt = rand.get_int(0, face_cnt) + 1;
    struct BfsFaceInfo {
    public:
        BfsFaceInfo(const CDT::Face_handle face) : face(face), parent(-1), children({ -1, -1, -1 }) {}

        CDT::Face_handle face;
        int parent;
        std::vector<int> children;  // Always has a length of 3.
    };
    std::vector<BfsFaceInfo> bfs_faces;
    bfs_faces.push_back(BfsFaceInfo(root));
    int front = 0;
    // Mark it as visited.
    bfs_faces[front].face->info().visited = true;
    while (static_cast<int>(bfs_faces.size()) < target_face_cnt) {
        // Visited its three children.
        for (int i = 0; i < 3; ++i) {
            const CDT::Face_handle new_face = bfs_faces[front].face->neighbor(i);
            if (new_face->info().in_domain() && !new_face->info().visited) {
                new_face->info().visited = true;
                BfsFaceInfo new_face_info(new_face);
                new_face_info.parent = front;
                bfs_faces[front].children[i] = static_cast<int>(bfs_faces.size());
                bfs_faces.push_back(new_face_info);
                if (static_cast<int>(bfs_faces.size()) == target_face_cnt) break;
            }
        }
        ++front;
    }

    // Generate the polygon.
    std::ostringstream oss;
    int current_face = 0;
    int current_edge = 0;
    // Each edge has three possible cases while being visited.
    // 1. It has not been visited yet = {v0, v1} has not been added to edge_info;
    // 2. One point is generated = edge_info[{v0, v1}] = { false, the other point };
    // 3. Both points have been generated = edge_info[{v0, v1}] = { true, p }.
    std::map<std::pair<int, int>, std::pair<bool, Point_3>> edge_info;
    bool enter = false;
    std::vector<bool> skipped(bfs_faces.size(), false);
    while (true) {
        // Regularize the key.
        const auto f = bfs_faces[current_face].face;
        const int v0 = f->vertex(cdt.ccw(current_edge))->id();
        const int v1 = f->vertex(cdt.cw(current_edge))->id();
        const int vmin = std::min(v0, v1);
        const int vmax = v0 + v1 - vmin;
        const Point_3 p0 = polyhedron.vertices()[v0];
        const Point_3 p1 = polyhedron.vertices()[v1];
        const auto key = std::make_pair(vmin, vmax);
        if (edge_info.find(key) == edge_info.end()) {
            CheckError(!enter, "Should be leaving this triangle.");
            const int next_face = bfs_faces[current_face].children[current_edge];
            bool skip = false;
            if (next_face != -1) {
                current_face = next_face;
                // Determine the next edge.
                current_edge = -1;
                for (int i = 0; i < 3; ++i) {
                    if (bfs_faces[current_face].face->vertex(i)->id() == v1) {
                        current_edge = i;
                        break;
                    }
                }
                CheckError(current_edge != -1, "Missing edges.");
                enter = false;
            } else {
                // Skip this edge with some probability.
                skip = !skipped[current_face] && rand.get_double() < 0.5;
                if (!skip) enter = true;
                else {
                    current_edge = cdt.ccw(current_edge);
                    enter = false;
                    skipped[current_face] = true;
                }
            }
            if (!skip) {
                // Generate a point.
                const Point_3 q0 = CGAL::barycenter(p0, rand.get_double(0.55, 0.95), p1);
                const Point_3 q1 = CGAL::barycenter(p0, rand.get_double(0.05, 0.45), p1);
                // They must be colinear.
                CheckError(CGAL::collinear(p0, p1, q0), "Expect colinearity.");
                CheckError(CGAL::collinear(p0, p1, q1), "Expect colinearity.");
                edge_info[key] = std::make_pair(false, q1);
                oss << q0 << " ";
            }
        } else if (!edge_info[key].first) {
            oss << edge_info[key].second << " ";
            edge_info[key].first = true;
            if (enter) {
                current_edge = cdt.ccw(current_edge);
            } else {
                int next_face = bfs_faces[current_face].parent;
                if (next_face != -1) {
                    current_face = next_face;
                    // Determine the next edge.
                    current_edge = -1;
                    for (int i = 0; i < 3; ++i) {
                        if (bfs_faces[current_face].face->vertex(i)->id() == v1) {
                            current_edge = i;
                            break;
                        }
                    }
                    CheckError(current_edge != -1, "Missing edges.");
                } else {
                    break;
                }
            }
            enter = false;
        } else {
            // The cycle closes.
            break;
        }
    }
    return oss.str();
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
    std::vector<Point_3> polygon(poly_dof);
    for (int i = 0; i < poly_dof; ++i) {
        iss >> polygon[i];
    }
    Vector_3 dir;
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