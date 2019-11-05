#include "core/nef3_wrapper.h"
#include "core/common.h"
#include "core/file_helper.h"

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
        const real x = CGAL::to_double(v_iter->point().x());
        const real y = CGAL::to_double(v_iter->point().y());
        const real z = CGAL::to_double(v_iter->point().z());
        Vector3r p(x, y, z);
        vertices_.push_back(p);
    }

    half_edges_.clear();
    half_edge_twins_.clear();
    const int half_edge_num = static_cast<int>(poly_.number_of_halfedges());
    half_edges_.reserve(half_edge_num);
    half_edge_twins_.reserve(half_edge_num);
    Nef_polyhedron::Halfedge_const_iterator e_iter;
    for (e_iter = poly_.halfedges_begin(); e_iter != poly_.halfedges_end(); ++e_iter) {
        const real sx = CGAL::to_double(e_iter->source()->point().x());
        const real sy = CGAL::to_double(e_iter->source()->point().y());
        const real sz = CGAL::to_double(e_iter->source()->point().z());
        const int source_idx = GetVertexIndex(Vector3r(sx, sy, sz));

        const real tx = CGAL::to_double(e_iter->target()->point().x());
        const real ty = CGAL::to_double(e_iter->target()->point().y());
        const real tz = CGAL::to_double(e_iter->target()->point().z());
        const int target_idx = GetVertexIndex(Vector3r(tx, ty, tz));
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

void Nef3Wrapper::Regularize(const Nef3Wrapper& other) {
    // This function does all the magic!
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
        const real x = v.x(), y = v.y(), z = v.z();
        std::cout << "v" << idx << "\t" << x << "\t" << y << "\t" << z << std::endl;
        ++idx;
    }
}

void Nef3Wrapper::ListEdges() const {
    std::cout << "Edge number " << half_edges_.size() << std::endl;
    int idx = 0;
    for (const auto& e : half_edges_) {
        std::cout << "e" << idx << "\tv" << e.first << "\tv" << e.second << "\ttwin\t"
            << "e" << half_edge_twins_[idx] << std::endl;
        ++idx;
    }
}

void Nef3Wrapper::ListFacets() const {
    std::cout << "Face number " << half_facets_.size() << std::endl;
    int idx = 0;
    for (const auto& f : half_facets_) {
        std::cout << "f" << idx << "\t" << f.size() << std::endl;
        for (const auto& fc : f) {
            for (const int v : fc) std::cout << "v" << v << "\t";
            std::cout << std::endl;
        }
        std::cout << "twin\t" << "f" << half_facet_twins_[idx] << std::endl;
        ++idx;
    }
}

const int Nef3Wrapper::GetVertexIndex(const Vector3r& vertex) const {
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