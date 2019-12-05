#include "core/nef3_wrapper.h"
#include "CGAL/AABB_tree.h"
#include "CGAL/AABB_traits.h"
#include "CGAL/AABB_face_graph_triangle_primitive.h"
#include "core/common.h"
#include "core/file_helper.h"
#include "core/triangulation.h"

// Ray tracing.
typedef Exact_kernel::Segment_3 Segment;
typedef Exact_kernel::Ray_3 Ray;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Exact_kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
typedef Tree::Primitive_id Primitive_id;

static const bool CanPermute(const std::vector<int>& cycle1, const std::vector<int>& cycle2) {
    if (cycle1.size() != cycle2.size()) return false;
    const int num = static_cast<int>(cycle1.size());
    int start_idx = -1;
    for (int i = 0; i < num; ++i)
        if (cycle2[i] == cycle1[0]) {
            start_idx = i;
            break;
        }
    if (start_idx == -1) return false;
    for (int i = 0; i < num; ++i) {
        const int j = (start_idx + i) % num;
        if (cycle1[i] != cycle2[j]) return false;
    }
    return true;
}

static const bool CanPermute(const std::vector<std::vector<int>>& cycles1, const std::vector<std::vector<int>>& cycles2) {
    if (cycles1.size() != cycles2.size()) return false;
    const int num_cycles = static_cast<int>(cycles1.size());
    std::vector<bool> mapped(num_cycles, false);
    for (int i = 0; i < num_cycles; ++i) {
        bool same = false;
        for (int j = 0; j < num_cycles; ++j) {
            if (mapped[j]) continue;
            if (CanPermute(cycles1[i], cycles2[j])) {
                mapped[j] = same = true;
                break;
            }
        }
        if (!same) return false;
    }
    return true;
}

const bool Nef3Wrapper::IsOutwardHalfFacet(const int fid, const int vc_idx) const {
    // Step 1: BVH.
    Polyhedron poly;
    poly_.convert_to_polyhedron(poly);
    Tree tree(faces(poly).first, faces(poly).second, poly);

    // Step 2: Compute the bounding box.
    Vector3r box_min = ToEigenVector3r(vertices_[0]);
    Vector3r box_max = box_min;
    for (const auto& v : vertices_) {
        const Vector3r p = ToEigenVector3r(v);
        box_min = box_min.cwiseMin(p);
        box_max = box_max.cwiseMax(p);
    }
    const Vector3r center = (box_min + box_max) / 2;
    // Big enough radius.
    const real radius = (box_max - box_min).norm() + 1.0;

    // Step 3: Triangulation. The goal is to find a random point on the face.
    std::vector<Point_3> polygon;
    for (const int i : half_facets_[fid][vc_idx]) {
        polygon.push_back(vertices_[i]);
    }
    CDT cdt = Triangulate(polygon, half_facets_[fid][vc_idx]);

    CDT::Face_handle root;
    for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
        if (fit->info().in_domain()) {
            root = CDT::Face_handle(fit);
            break;
        }
    }
    std::vector<Point_3> pts(3);
    for (int i = 0; i < 3; ++i) pts[i] = vertices_[root->vertex(i)->id()];
    const Vector_3 normal = GetPolygonNormal(polygon);

    // Step 4: Ray tracing.
    CGAL::Random rand;
    int inward_cnt = 0, outward_cnt = 0;
    int test_num = 7;
    while (!(inward_cnt * 2 > test_num || outward_cnt * 2 > test_num)) {
        const real w0 = rand.get_double(0, 0.4);
        const real w1 = rand.get_double(0, 0.4);
        const Point_3 pt = CGAL::barycenter(pts[0], w0, pts[1], w1, pts[2]);
        // Randomly generate the origin of the ray.
        const real theta = rand.get_double() * 3.14;
        const real phi = rand.get_double() * 2 * 3.14;
        const Vector3r ray_origin = center + radius * Vector3r(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
        const Point_3 origin(ray_origin.x(), ray_origin.y(), ray_origin.z());
        Segment query(origin, pt);
        // Count the number of intesections.
        const int cnt = tree.number_of_intersected_primitives(query);
        const Vector_3 dir = pt - origin;
        if ((dir * normal) * (ToReal(cnt % 2) - ToReal(0.5)) < 0) ++outward_cnt;
        else ++inward_cnt;
    }
    return outward_cnt * 2 > test_num;
}

void Nef3Wrapper::Load(const std::string& file_name) {
    std::ifstream in_file(file_name);
    poly_ = Nef_polyhedron();
    in_file >> poly_;
    CheckError(poly_.is_simple(), "The polyhedron is not a 2-manifold.");
    SyncDataStructure();
}

void Nef3Wrapper::SyncDataStructure() {
    std::ostringstream oss;
    oss << poly_;
    const std::vector<std::string> lines = SplitString(oss.str(), '\n');

    // Now construct the data structure. The code below is quite inefficient. However, since we are almost always
    // dealing with tiny scenes, I think the inefficiency is tolerable.
    vertices_.clear();
    const int vertex_num = static_cast<int>(poly_.number_of_vertices());
    vertices_.reserve(vertex_num);
    Nef_polyhedron::Vertex_const_iterator v_iter;
    for (v_iter = poly_.vertices_begin(); v_iter != poly_.vertices_end(); ++v_iter) {
        vertices_.push_back(v_iter->point());
    }

    half_edges_.clear();
    half_edge_twins_.clear();
    const int half_edge_num = static_cast<int>(poly_.number_of_halfedges());
    half_edges_.reserve(half_edge_num);
    half_edge_twins_.reserve(half_edge_num);
    Nef_polyhedron::Halfedge_const_iterator e_iter;
    for (e_iter = poly_.halfedges_begin(); e_iter != poly_.halfedges_end(); ++e_iter) {
        const int source_idx = GetVertexIndex(e_iter->source()->point());
        const int target_idx = GetVertexIndex(e_iter->target()->point());
        half_edges_.push_back(std::make_pair(source_idx, target_idx));
    }
    // Construct the twin edges.
    for (int i = 0; i < half_edge_num; ++i) {
        half_edge_twins_[i] = GetHalfEdgeIndex(half_edges_[i].second, half_edges_[i].first);
    }

    // Construct the face not from the polyhedron but from the file (CGAL's data structure is quite hard to understand...).
    half_facets_.clear();
    half_facet_twins_.clear();
    const int half_facet_num = static_cast<int>(poly_.number_of_halffacets());
    half_facets_.reserve(half_facet_num);
    half_facet_twins_.resize(half_facet_num, -1);

    // Now parse the input file.
    int line_idx = 0;
    int shalf_edge_num = 0;
    int volume_num = 0;
    std::vector<int> shalf_edge_to_vertex;
    int vertex_begin = 0;
    int half_edge_begin = 0;
    int half_facet_begin = 0;
    int volume_begin = 0;
    int shalf_edge_begin = 0;
    std::vector<std::vector<int>> shalf_edge_in_half_facets(half_facet_num);
    std::vector<int> shalf_edge_next;
    for (const auto& line : lines) {
        if (line_idx < 2) {
            // Header. Do nothing.
        } else if (line_idx == 2) {
            CheckError(StartsWith(line, "vertices"), "Expect to see vertices.");
            CheckError(std::stoi(SplitString(line)[1]) == vertex_num, "Inconsistent vertex number.");
        } else if (line_idx == 3) {
            CheckError(StartsWith(line, "halfedges"), "Expect to see halfedges.");
            CheckError(std::stoi(SplitString(line)[1]) == half_edge_num, "Inconsistent halfedge number.");
        } else if (line_idx == 4) {
            CheckError(StartsWith(line, "facets"), "Expect to see facets.");
            CheckError(std::stoi(SplitString(line)[1]) == half_facet_num, "Inconsistent facets number.");
        } else if (line_idx == 5) {
            CheckError(StartsWith(line, "volumes"), "Expect to see volumes.");
            volume_num = std::stoi(SplitString(line)[1]);
        } else if (line_idx == 6) {
            CheckError(StartsWith(line, "shalfedges"), "Expect to see shalfedges.");
            shalf_edge_num = std::stoi(SplitString(line)[1]);
            shalf_edge_to_vertex.clear();
            shalf_edge_to_vertex.resize(shalf_edge_num, -1);
            shalf_edge_next.clear();
            shalf_edge_next.resize(shalf_edge_num, -1);
        } else if (line_idx == 7) {
            CheckError(StartsWith(line, "shalfloops"), "Expect to see shalfloops.");
        } else if (line_idx == 8) {
            CheckError(StartsWith(line, "sfaces"), "Expect to see sfaces.");
            vertex_begin = 9;
            half_edge_begin = vertex_begin + vertex_num;
            half_facet_begin = half_edge_begin + half_edge_num;
            volume_begin = half_facet_begin + half_facet_num;
            shalf_edge_begin = volume_begin + volume_num;
        } else if (line_idx >= vertex_begin && line_idx < half_edge_begin) {
            // Parse vertices.
            std::istringstream iss(line);
            int vid, dummy;
            int sbegin, send;
            char ch;
            iss >> vid;
            CheckError(vid == line_idx - vertex_begin, "vid mismatches.");
            iss >> ch;  // '{'
            iss >> dummy >> dummy >> ch; // Half edges.
            iss >> sbegin >> send;
            for (int i = sbegin; i <= send; ++i) {
                shalf_edge_to_vertex[i] = vid;
            }
        } else if (line_idx >= half_edge_begin && line_idx < half_facet_begin) {
            // Half edges. Do nothing for now.
        } else if (line_idx >= half_facet_begin && line_idx < volume_begin) {
            // Half facets. Do the real work here.
            // Sample line: 10 { 11, 12 83 , , 0 | 0 1 0 -1 } 1
            std::string new_line = line.substr(0, line.find(',', line.find(',') + 1));
            std::istringstream iss(new_line);
            int fid;
            iss >> fid;
            CheckError(fid == line_idx - half_facet_begin, "fid mismatches.");
            char ch;
            iss >> ch;
            iss >> half_facet_twins_[fid];
            new_line = new_line.substr(new_line.find(',') + 1);
            const std::vector<std::string> words = SplitString(new_line);
            shalf_edge_in_half_facets[fid].clear();
            for (const auto& w : words) shalf_edge_in_half_facets[fid].push_back(std::stoi(w));
        } else if (line_idx >= volume_begin && line_idx < shalf_edge_begin) {
            // Volumes. Skip.
        } else if (line_idx >= shalf_edge_begin && line_idx < shalf_edge_begin + shalf_edge_num) {
            // Shalf edges.
            // Sample line: 0 { 1, 4, 2, 0, 1, 26, 18, 6 | 0 1 0 0 } 1
            std::istringstream iss(line);
            int seid;
            iss >> seid;
            CheckError(seid == line_idx - shalf_edge_begin, "seid mismatches.");
            char ch;
            iss >> ch;  // '{'
            int dummy;
            for (int i = 0; i < 6; ++i) iss >> dummy >> ch;
            iss >> shalf_edge_next[seid];
        } else {
            // We don't care about anything left. Skip.
        }
        ++line_idx;
    }
    for (const auto& fc : shalf_edge_in_half_facets) {
        std::vector<std::vector<int>> fc_idx;
        for (const int v : fc) {
            std::vector<int> vc;
            vc.push_back(shalf_edge_to_vertex[v]);
            int v_next = shalf_edge_next[v];
            while (v_next != v) {
                vc.push_back(shalf_edge_to_vertex[v_next]);
                v_next = shalf_edge_next[v_next];
            }
            fc_idx.push_back(vc);
        }
        half_facets_.push_back(fc_idx);
    }
    ComputeFacetOrientation();
    ComputeFacetNormal();

    vertices_match_target_.clear();
    vertices_match_target_.resize(vertices_.size(), false);
    half_edges_match_target_.clear();
    half_edges_match_target_.resize(half_edges_.size(), false);
    half_facets_match_target_.clear();
    half_facets_match_target_.resize(half_facets_.size(), false);
}

void Nef3Wrapper::ComputeFacetNormal() {
    half_facet_normals_.clear();
    const int facet_num = static_cast<int>(half_facets_.size());
    half_facet_normals_.resize(facet_num, Vector_3(1, 0, 0));
    std::vector<bool> solved(facet_num, false);
    for (int fi = 0; fi < facet_num; ++fi) {
        if (solved[fi]) continue;
        std::vector<std::vector<Point_3>> polygon;
        for (const auto& vc : half_facets_[fi]) {
            std::vector<Point_3> p;
            for (const int i : vc) p.push_back(vertices_[i]);
            polygon.push_back(p);
        }
        CDT cdt = Triangulate(polygon, half_facets_[fi]);

        bool found = false;
        for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end() && !found; ++fit) {
            if (fit->info().in_domain()) {
                // Check if it has a constraint edge.
                CDT::Face_handle fh = CDT::Face_handle(fit);
                for (int i = 0; i < 3; ++i) {
                    CDT::Edge e(fh, i);
                    if (cdt.is_constrained(e)) {
                        found = true;
                        const int v0 = fh->vertex(cdt.cw(i))->id();
                        const int v1 = fh->vertex(cdt.ccw(i))->id();
                        // (v0, v1) must be an edge.
                        const int v2 = fh->vertex(0)->id() + fh->vertex(1)->id() + fh->vertex(2)->id() - v0 - v1;
                        // Figure out the direction of v0 and v1.
                        bool swap = false;
                        bool visited = false;
                        for (const auto& vc : half_facets_[fi]) {
                            const int vc_len = static_cast<int>(vc.size());
                            for (int j = 0; j < vc_len; ++j) {
                                if (vc[j] == v0) {
                                    const int j1 = vc[(j + 1) % vc_len];
                                    const int j0 = vc[(j + vc_len - 1) % vc_len];
                                    CheckError(j1 == v1 || j0 == v1, "Missing an edge in polygon.");
                                    swap = j0 == v1;
                                    visited = true;
                                    break;
                                }
                            }
                            if (visited) break;
                        }
                        CheckError(visited, "Vertex does not exist in the polyon.");
                        const Point_3 p0 = swap ? vertices_[v1] : vertices_[v0];
                        const Point_3 p1 = swap ? vertices_[v0] : vertices_[v1];
                        const Point_3 p2 = vertices_[v2];
                        half_facet_normals_[fi] = CGAL::normal(p0, p1, p2);
                        solved[fi] = solved[half_facet_twins_[fi]] = true;
                        half_facet_normals_[half_facet_twins_[fi]] = -half_facet_normals_[fi];
                        break;
                    }
                }
            }
        }
        CheckError(found, "This face does not contain a triangle adjacent to its edges.");
    }
}

void Nef3Wrapper::ComputeFacetOrientation() {
    half_facet_outwards_.clear();
    const int facet_num = static_cast<int>(half_facets_.size());
    half_facet_outwards_.resize(facet_num, true);
    // Step 1: for each edge, figure out its two neighbor facets.
    std::map<std::pair<int, int>, std::pair<int, int>> edge_map;
    for (int fi = 0; fi < facet_num; ++fi) {
        for (const auto& vc : half_facets_[fi]) {
            const int vc_len = static_cast<int>(vc.size());
            for (int i = 0; i < vc_len; ++i) {
                const int j = (i + 1) % vc_len;
                const int e0 = vc[i], e1 = vc[j];
                const auto key = std::make_pair(e0, e1);
                if (edge_map.find(key) == edge_map.end()) {
                    edge_map[key] = std::make_pair(fi, -1);
                } else {
                    auto& val = edge_map[key];
                    CheckError(val.second == -1, "This polyhedron is broken.");
                    val.second = fi;
                }
            }
        }
    }
    // Step 2: BFS.
    std::vector<bool> visited(facet_num, false);
    while (true) {
        // In the remaining unvisited facets, pick the one that has a single vertex cycle.
        int root_facet = -1;
        for (int i = 0; i < facet_num; ++i) {
            if (visited[i]) continue;
            if (static_cast<int>(half_facets_[i].size()) == 1) {
                root_facet = i;
                break;
            } else {
                --root_facet;
            }
        }
        // root_facet == -1: Done.
        // root_facet >= 0: Unfinished.
        // root_facet < -1: The shape is not a 2-manifold?
        if (root_facet == -1) break;
        CheckError(root_facet >= 0, "Each remaining unvisited facet has at least one hole.");
        const bool root_outward = IsOutwardHalfFacet(root_facet, 0);
        std::deque<int> queue;
        queue.push_back(root_facet);
        visited[root_facet] = true;
        half_facet_outwards_[root_facet] = root_outward;
        half_facet_outwards_[half_facet_twins_[root_facet]] = !root_outward;
        while (!queue.empty()) {
            const int front = queue.front();
            queue.pop_front();
            const bool front_outward = half_facet_outwards_[front];
            // Loop over its edges.
            for (const auto& vc : half_facets_[front]) {
                const int vc_len = static_cast<int>(vc.size());
                for (int i = 0; i < vc_len; ++i) {
                    const int j = (i + 1) % vc_len;
                    const int e0 = vc[i], e1 = vc[j];
                    // Find my neighbor.
                    const auto key = std::make_pair(e0, e1);
                    const auto& val = edge_map.at(key);
                    const int next_f = val.first == front ? val.second : val.first;
                    if (visited[next_f]) continue;
                    // Add new faces.
                    visited[next_f] = true;
                    half_facet_outwards_[next_f] = !front_outward;
                    half_facet_outwards_[half_facet_twins_[next_f]] = front_outward;
                    queue.push_back(next_f);
                }
            }
        }
    }
}

void Nef3Wrapper::Save(const std::string& file_name) {
    // Check the type of the file name.
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && (name_and_ext[1] == "off" || name_and_ext[1] == "nef3"), "Invalid file name.");
    CheckError(poly_.is_simple(), "The current scene is not a 2-manifold.");
    PrepareToCreateFile(file_name);

    const std::string ext = name_and_ext[1];
    if (ext == "off") {
        Polyhedron poly;
        poly_.convert_to_polyhedron(poly);
        std::ofstream output(file_name);
        write_off(output, poly);
    } else {
        std::ofstream output(file_name);
        output << poly_;
    }
}

template <class HDS>
class BuildExtrusion : public CGAL::Modifier_base<HDS> {
public:
    void SetExtrusionInfo(const std::vector<Point_3>& polygon,
        const Aff_transformation_3& dir) {
        polygon_ = polygon;
        dir_ = dir;
    }

    void operator()(HDS& hds) {
        // Check the orientation of the polygon.
        // TODO: maybe CGAL has done this already?
        const int poly_dof = static_cast<int>(polygon_.size());
        CheckError(poly_dof >= 3, "You need at least three points to define a polygon");
        // Check if they are on the same plane.
        const Point_3& v0 = polygon_[0];
        const Point_3& v1 = polygon_[1];
        int third = 2;
        while (third < poly_dof) {
            const Point_3& v2 = polygon_[third];
            if (!CGAL::collinear(v0, v1, v2)) break;
            ++third;
        }
        CheckError(third < poly_dof, "Polygon is degenerated into a line.");
        const Point_3& v2 = polygon_[third];
        for (int i = third + 1; i < poly_dof; ++i) {
            CheckError(CGAL::coplanar(v0, v1, v2, polygon_[i]), "Polygon is not defined on a single plane.");
        }
        // End of the sanity check.
        std::vector<Vector3r> offset_polygon;
        for (const auto& p : polygon_) {
            offset_polygon.push_back(Nef3Wrapper::ToEigenVector3r(p));
        }
        for (auto& v : offset_polygon) v -= offset_polygon[0];
        real vol = 0;
        const Vector3r d = Nef3Wrapper::ToEigenVector3r(dir_(polygon_[0]) - polygon_[0]);
        for (int i = 0; i < poly_dof; ++i) {
            const int next_i = (i + 1) % poly_dof;
            const Vector3r v0 = offset_polygon[i], v1 = offset_polygon[next_i];
            vol += v0.cross(v1).dot(d);
        }
        const bool reversed = vol < 0;

        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);
        builder.begin_surface(2 * poly_dof, 2 + poly_dof, 6 * poly_dof);
        // Add points.
        for (const auto& p : polygon_) {
            builder.add_vertex(p);
        }
        // Add points in the second layer.
        for (const auto& p : polygon_) {
            builder.add_vertex(dir_(p));
        }
        // Add faces.
        if (reversed) {
            builder.begin_facet();
            for (int i = 0; i < poly_dof; ++i) builder.add_vertex_to_facet(i);
            builder.end_facet();

            builder.begin_facet();
            for (int i = poly_dof - 1; i >= 0; --i) builder.add_vertex_to_facet(i + poly_dof);
            builder.end_facet();

            for (int i = 0; i < poly_dof; ++i) {
                const int j = (i + 1) % poly_dof;
                builder.begin_facet();
                builder.add_vertex_to_facet(j);
                builder.add_vertex_to_facet(i);
                builder.add_vertex_to_facet(i + poly_dof);
                builder.add_vertex_to_facet(j + poly_dof);
                builder.end_facet();
            }
        } else {
            builder.begin_facet();
            for (int i = 0; i < poly_dof; ++i) builder.add_vertex_to_facet(i + poly_dof);
            builder.end_facet();

            builder.begin_facet();
            for (int i = poly_dof - 1; i >= 0; --i) builder.add_vertex_to_facet(i);
            builder.end_facet();

            for (int i = 0; i < poly_dof; ++i) {
                const int j = (i + 1) % poly_dof;
                builder.begin_facet();
                builder.add_vertex_to_facet(i);
                builder.add_vertex_to_facet(j);
                builder.add_vertex_to_facet(j + poly_dof);
                builder.add_vertex_to_facet(i + poly_dof);
                builder.end_facet();
            }
        }
        builder.end_surface();
    }

private:
    std::vector<Point_3> polygon_;
    Aff_transformation_3 dir_;
};

const Nef_polyhedron Nef3Wrapper::BuildExtrusionFromData(const std::vector<Point_3>& polygon,
    const Aff_transformation_3& dir) const {
    // Check if data are valid.
    CheckError(polygon.size() >= 3, "You need at least three points to define a polygon.");
    const int n = static_cast<int>(polygon.size());
    const Point_3& v0 = polygon[0];
    const Point_3& v1 = polygon[1];
    int next = -1;
    for (int i = 2; i < n; ++i) {
        if (CGAL::collinear(v0, v1, polygon[i])) continue;
        next = i;
        break;
    }
    CheckError(next != -1, "Polygon is degenerated into a line.");
    const Point_3 v0_transformed = dir(v0);
    CheckError(!CGAL::coplanar(v0, v1, polygon[next], v0_transformed), "Extrusion is degenerated because dir is parallel to polygon.");

    Polyhedron poly;
    BuildExtrusion<HalfedgeDS> extrusion;
    extrusion.SetExtrusionInfo(polygon, dir);
    poly.delegate(extrusion);
    return Nef_polyhedron(poly);
}

const Nef_polyhedron Nef3Wrapper::BuildExtrusionFromRef(const int f_idx, const int loop_idx,
    const int v_source, const int v_target) const {
    std::vector<Point_3> polygon;
    for (const int v : half_facets_[f_idx][loop_idx]) polygon.push_back(vertices_[v]);
    Aff_transformation_3 dir(CGAL::TRANSLATION, vertices_[v_target] - vertices_[v_source]);
    return BuildExtrusionFromData(polygon, dir);
}

void Nef3Wrapper::Regularize(const Nef3Wrapper& other, const real eps) {
    std::fill(vertices_match_target_.begin(), vertices_match_target_.end(), false);
    std::fill(half_edges_match_target_.begin(), half_edges_match_target_.end(), false);
    std::fill(half_facets_match_target_.begin(), half_facets_match_target_.end(), false);

    // TODO: should we use eps?
    const int vertex_num = static_cast<int>(vertices_.size());
    std::vector<int> old_to_new(vertex_num, -1), new_to_old(vertex_num, -1);
    // old_to_new_vertex_mapping must be a permutation.
    const int target_vertex_num = other.GetVertexNumber();
    for (int i = 0; i < vertex_num; ++i) {
        for (int j = 0; j < target_vertex_num; ++j) {
            if (vertices_[i] == other.vertices()[j]) {
                CheckError(old_to_new[i] == -1 && new_to_old[j] == -1, "Vertices are duplicated.");
                old_to_new[i] = j;
                new_to_old[j] = i;
                vertices_match_target_[j] = true;
                break;
            }
        }
    }
    std::vector<int> unclaimed;
    for (int i = 0; i < vertex_num; ++i) {
        if (new_to_old[i] == -1) unclaimed.push_back(i);
    }
    int cnt = 0;
    for (int i = 0; i < vertex_num; ++i) {
        if (old_to_new[i] != -1) continue;
        old_to_new[i] = unclaimed[cnt];
        new_to_old[unclaimed[cnt]] = i;
        ++cnt;
    }
    std::vector<Point_3> new_vertices(vertex_num);
    for (int i = 0; i < vertex_num; ++i) {
        new_vertices[old_to_new[i]] = vertices_[i];
    }
    new_vertices.swap(vertices_);

    // Update half edges.
    const int edge_num = static_cast<int>(half_edges_.size());
    std::vector<int> old_to_new_edges(edge_num, -1), new_to_old_edges(edge_num, -1);
    const int target_edge_num = other.GetHalfEdgeNumber();
    for (int i = 0; i < edge_num; ++i) {
        for (int j = 0; j < target_edge_num; ++j) {
            const int first_i = old_to_new[half_edges_[i].first];
            const int second_i = old_to_new[half_edges_[i].second];
            const int first_other = other.half_edges()[j].first;
            const int second_other = other.half_edges()[j].second;
            if (vertices_match_target_[first_i] && vertices_match_target_[second_i] &&
                first_i == first_other && second_i == second_other) {
                    old_to_new_edges[i] = j;
                    new_to_old_edges[j] = i;
                    half_edges_match_target_[j] = true;
                    break;
                }
        }
    }
    unclaimed.clear();
    for (int i = 0; i < edge_num; ++i) {
        if (new_to_old_edges[i] == -1) unclaimed.push_back(i);
    }
    cnt = 0;
    for (int i = 0; i < edge_num; ++i) {
        if (old_to_new_edges[i] != -1) continue;
        old_to_new_edges[i] = unclaimed[cnt];
        new_to_old_edges[unclaimed[cnt]] = i;
        ++cnt;
    }
    std::vector<std::pair<int, int>> new_half_edges(edge_num);
    std::vector<int> new_half_edge_twins(edge_num, -1);
    for (int i = 0; i < edge_num; ++i) {
        new_half_edges[old_to_new_edges[i]] = std::make_pair(old_to_new[half_edges_[i].first], old_to_new[half_edges_[i].second]);
        new_half_edge_twins[old_to_new_edges[i]] = old_to_new_edges[half_edge_twins_[i]];
    }
    new_half_edges.swap(half_edges_);
    new_half_edge_twins.swap(half_edge_twins_);

    // Update half facets.
    for (auto& f : half_facets_)
        for (auto& vc : f)
            for (int& v : vc) v = old_to_new[v];

    const int facet_num = static_cast<int>(half_facets_.size());
    std::vector<int> old_to_new_facets(facet_num, -1), new_to_old_facets(facet_num, -1);
    const int target_facet_num = other.GetHalfFacetNumber();
    for (int i = 0; i < facet_num; ++i) {
        for (int j = 0; j < target_facet_num; ++j) {
            // Check if half_facets_[i] == other.half_facets()[j].
            bool all_covered = true;
            // First, it should have the same number of cycles.
            // Then, all vertices must match the target.
            for (const auto& vc : half_facets_[i])
                for (const int v : vc)
                    if (!vertices_match_target_[v]) {
                        all_covered = false;
                        break;
                    }
            if (!all_covered) continue;
            if (CanPermute(half_facets_[i], other.half_facets()[j])) {
                old_to_new_facets[i] = j;
                new_to_old_facets[j] = i;
                half_facets_match_target_[j] = true;
                break;
            }
        }
    }
    unclaimed.clear();
    for (int i = 0; i < facet_num; ++i) {
        if (new_to_old_facets[i] == -1) unclaimed.push_back(i);
    }
    cnt = 0;
    for (int i = 0; i < facet_num; ++i) {
        if (old_to_new_facets[i] != -1) continue;
        old_to_new_facets[i] = unclaimed[cnt];
        new_to_old_facets[unclaimed[cnt]] = i;
        ++cnt;
    }
    std::vector<std::vector<std::vector<int>>> new_half_facets(facet_num);
    std::vector<int> new_half_facet_twins(facet_num, -1);
    std::vector<bool> new_half_facet_outwards(facet_num, -1);
    std::vector<Vector_3> new_half_facet_normals(facet_num);
    for (int i = 0; i < facet_num; ++i) {
        new_half_facets[old_to_new_facets[i]] = half_facets_[i];
        new_half_facet_twins[old_to_new_facets[i]] = old_to_new_facets[half_facet_twins_[i]];
        new_half_facet_outwards[old_to_new_facets[i]] = half_facet_outwards_[i];
        new_half_facet_normals[old_to_new_facets[i]] = half_facet_normals_[i];
    }
    new_half_facets.swap(half_facets_);
    new_half_facet_twins.swap(half_facet_twins_);
    new_half_facet_outwards.swap(half_facet_outwards_);
    new_half_facet_normals.swap(half_facet_normals_);
}

void Nef3Wrapper::operator+=(const Nef_polyhedron& other) {
    poly_ = (poly_ + other).regularization();
    SyncDataStructure();
}

void Nef3Wrapper::operator-=(const Nef_polyhedron& other) {
    poly_ = (poly_ - other).regularization();
    SyncDataStructure();
}

void Nef3Wrapper::ListVertices() const {
    std::cout << "Vertex number " << vertices_.size() << std::endl;
    int idx = 0;
    for (const auto& v : vertices_) {
        if (vertices_match_target_[idx])
            std::cout << GreenHead() << "v" << idx << "\t" << v.exact() << GreenTail() << std::endl;
        else
            std::cout << "v" << idx << "\t" << v.exact() << std::endl;
        ++idx;
    }
}

void Nef3Wrapper::ListEdges() const {
    std::cout << "Edge number " << half_edges_.size() << std::endl;
    int idx = 0;
    for (const auto& e : half_edges_) {
        if (half_edges_match_target_[idx])
            std::cout << GreenHead() << "e" << idx << "\tv" << e.first << "\tv" << e.second << "\ttwin\t"
                << "e" << half_edge_twins_[idx] << GreenTail() << std::endl;
        else
            std::cout << "e" << idx << "\tv" << e.first << "\tv" << e.second << "\ttwin\t"
                << "e" << half_edge_twins_[idx] << std::endl;
        ++idx;
    }
}

void Nef3Wrapper::ListFacets() const {
    std::cout << "Face number " << half_facets_.size() << std::endl;
    int idx = 0;
    for (const auto& f : half_facets_) {
        if (half_facets_match_target_[idx]) {
            std::cout << GreenHead() << "f" << idx << "\t" << f.size() << "\t"
                << (half_facet_outwards_[idx] ? "outward" : "inward") << GreenTail() << std::endl;
            for (const auto& fc : f) {
                for (const int v : fc) std::cout << GreenHead() << "v" << v << "\t";
                std::cout << GreenTail() << std::endl;
            }
            std::cout << GreenHead() << "twin\t" << "f" << half_facet_twins_[idx] << GreenTail() << std::endl;
            std::cout << GreenHead() << "normal\t" << half_facet_normals_[idx].exact() << GreenTail() << std::endl;
        } else {
            std::cout << "f" << idx << "\t" << f.size() << "\t"
                << (half_facet_outwards_[idx] ? "outward" : "inward") << std::endl;
            for (const auto& fc : f) {
                for (const int v : fc) std::cout << "v" << v << "\t";
                std::cout << std::endl;
            }
            std::cout << "twin\t" << "f" << half_facet_twins_[idx] << std::endl;
            std::cout << "normal\t" << half_facet_normals_[idx].exact() << std::endl;
        }
        ++idx;
    }
}

const int Nef3Wrapper::GetVertexIndex(const Point_3& vertex) const {
    int idx = 0;
    for (const auto& v : vertices_) {
        if (v.x() == vertex.x() && v.y() == vertex.y() && v.z() == vertex.z()) break;
        ++idx;
    }
    return idx;
}

const int Nef3Wrapper::GetHalfEdgeIndex(const int source, const int target) const {
    int idx = 0;
    for (const auto& e : half_edges_) {
        if (e.first == source && e.second == target) break;
        ++idx;
    }
    return idx;
}

const Vector3r Nef3Wrapper::ToEigenVector3r(const Point_3& point) {
    Vector3r p(CGAL::to_double(point.x()), CGAL::to_double(point.y()), CGAL::to_double(point.z()));
    return p;
}

const Vector3r Nef3Wrapper::ToEigenVector3r(const Vector_3& vector) {
    Vector3r p(CGAL::to_double(vector.x()), CGAL::to_double(vector.y()), CGAL::to_double(vector.z()));
    return p;
}
