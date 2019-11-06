#include "core/nef3_wrapper.h"
#include "core/common.h"
#include "core/file_helper.h"

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

    vertices_match_target_.clear();
    vertices_match_target_.resize(vertices_.size(), false);
    half_edges_match_target_.clear();
    half_edges_match_target_.resize(half_edges_.size(), false);
    half_facets_match_target_.clear();
    half_facets_match_target_.resize(half_facets_.size(), false);
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
    BuildExtrusion(const Nef3Wrapper& parent) : parent_(parent) {}

    void SetExtrusionInfo(const int f_idx, const int loop_idx, const int v_source, const int v_target) {
        f_idx_ = f_idx;
        loop_idx_ = loop_idx;
        v_source_ = v_source;
        v_target_ = v_target;
    }

    void operator()(HDS& hds) {
        // Check the orientation of the polygon.
        // TODO: maybe CGAL has done this already?
        const int poly_dof = static_cast<int>(parent_.half_facets()[f_idx_][loop_idx_].size());
        std::vector<Vector3r> offset_polygon;
        for (const int vid : parent_.half_facets()[f_idx_][loop_idx_]) {
            offset_polygon.push_back(parent_.ToEigenVector3r(parent_.vertices()[vid]));
        }
        for (auto& v : offset_polygon) v -= offset_polygon[0];
        real vol = 0;
        const Vector3r dir = parent_.ToEigenVector3r(parent_.vertices()[v_target_])
            - parent_.ToEigenVector3r(parent_.vertices()[v_source_]);
        for (int i = 0; i < poly_dof; ++i) {
            const int next_i = (i + 1) % poly_dof;
            const Vector3r v0 = offset_polygon[i], v1 = offset_polygon[next_i];
            vol += v0.cross(v1).dot(dir);
        }    
        const bool reversed = vol < 0;

        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);
        builder.begin_surface(2 * poly_dof, 2 + poly_dof, 6 * poly_dof);
        // Add points.
        for (const int vid : parent_.half_facets()[f_idx_][loop_idx_]) {
            builder.add_vertex(parent_.vertices()[vid]);
        }
        // Add points in the second layer.
        const auto& ext_source = parent_.vertices()[v_source_];
        const auto& ext_target = parent_.vertices()[v_target_];
        Aff_transformation_3 shift(CGAL::TRANSLATION, Vector_3(ext_target - ext_source));
        for (const int vid : parent_.half_facets()[f_idx_][loop_idx_]) {
            builder.add_vertex(shift(parent_.vertices()[vid]));
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
    const Nef3Wrapper& parent_;
    int f_idx_, loop_idx_, v_source_, v_target_;
};

const Nef_polyhedron Nef3Wrapper::BuildExtrusionFromRef(const int f_idx, const int loop_idx,
    const int v_source, const int v_target) const {
    Polyhedron poly;
    BuildExtrusion<HalfedgeDS> extrusion(*this);
    extrusion.SetExtrusionInfo(f_idx, loop_idx, v_source, v_target);
    poly.delegate(extrusion);
    return Nef_polyhedron(poly);
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
    std::vector<Exact_kernel::Point_3> new_vertices(vertex_num);
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
    for (int i = 0; i < facet_num; ++i) {
        new_half_facets[old_to_new_facets[i]] = half_facets_[i];
        new_half_facet_twins[old_to_new_facets[i]] = old_to_new_facets[half_facet_twins_[i]];
    }
    new_half_facets.swap(half_facets_);
    new_half_facet_twins.swap(half_facet_twins_);
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
        const real x = CGAL::to_double(v.x()), y = CGAL::to_double(v.y()), z = CGAL::to_double(v.z());
        if (vertices_match_target_[idx])
            std::cout << GreenHead() << "v" << idx << "\t" << x << "\t" << y << "\t" << z << GreenTail() << std::endl;
        else
            std::cout << "v" << idx << "\t" << x << "\t" << y << "\t" << z << std::endl;
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
            std::cout << GreenHead() << "f" << idx << "\t" << f.size() << GreenTail() << std::endl;
            for (const auto& fc : f) {
                for (const int v : fc) std::cout << GreenHead() << "v" << v << "\t";
                std::cout << GreenTail() << std::endl;
            }
            std::cout << GreenHead() << "twin\t" << "f" << half_facet_twins_[idx] << GreenTail() << std::endl;
        } else {
            std::cout << "f" << idx << "\t" << f.size() << std::endl;
            for (const auto& fc : f) {
                for (const int v : fc) std::cout << "v" << v << "\t";
                std::cout << std::endl;
            }
            std::cout << "twin\t" << "f" << half_facet_twins_[idx] << std::endl;
        }
        ++idx;
    }
}

const int Nef3Wrapper::GetVertexIndex(const Exact_kernel::Point_3& vertex) const {
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

const Vector3r Nef3Wrapper::ToEigenVector3r(const Exact_kernel::Point_3& point) {
    Vector3r p(CGAL::to_double(point.x()), CGAL::to_double(point.y()), CGAL::to_double(point.z()));
    return p;
}