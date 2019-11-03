#include "core/scene.h"
#include "core/common.h"
#include "core/file_helper.h"

Scene::Scene() {}

void Scene::LoadScene(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    std::ifstream input(file_name);
    objects_ = Nef_polyhedron();    // Clear the original polyhedron.
    input >> objects_;
    CheckError(objects_.is_simple(), "The current scene is not a 2-manifold.");
}

void Scene::LoadTarget(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && name_and_ext[1] == "nef3", "Invalid file name.");
    std::ifstream input(file_name);
    target_ = Nef_polyhedron();
    input >> target_;
    CheckError(target_.is_simple(), "The target is not a 2-manifold.");

    // Now construct the data structure. The code below is quite inefficient. However, since we are almost always
    // dealing with tiny scenes, I think the inefficiency is tolerable.
    target_vertices_.clear();
    const int vertex_num = static_cast<int>(target_.number_of_vertices());
    target_vertices_.reserve(vertex_num);
    Nef_polyhedron::Vertex_const_iterator v_iter;
    for (v_iter = target_.vertices_begin(); v_iter != target_.vertices_end(); ++v_iter) {
        const real x = CGAL::to_double(v_iter->point().x());
        const real y = CGAL::to_double(v_iter->point().y());
        const real z = CGAL::to_double(v_iter->point().z());
        Vector3r p(x, y, z);
        target_vertices_.push_back(p);
    }

    target_half_edges_.clear();
    target_half_edge_twins_.clear();
    const int half_edge_num = static_cast<int>(target_.number_of_halfedges());
    target_half_edges_.reserve(half_edge_num);
    target_half_edge_twins_.reserve(half_edge_num);
    Nef_polyhedron::Halfedge_const_iterator e_iter;
    for (e_iter = target_.halfedges_begin(); e_iter != target_.halfedges_end(); ++e_iter) {
        const real sx = CGAL::to_double(e_iter->source()->point().x());
        const real sy = CGAL::to_double(e_iter->source()->point().y());
        const real sz = CGAL::to_double(e_iter->source()->point().z());
        const int source_idx = GetVertexIndex(Vector3r(sx, sy, sz));

        const real tx = CGAL::to_double(e_iter->target()->point().x());
        const real ty = CGAL::to_double(e_iter->target()->point().y());
        const real tz = CGAL::to_double(e_iter->target()->point().z());
        const int target_idx = GetVertexIndex(Vector3r(tx, ty, tz));
        target_half_edges_.push_back(std::make_pair(source_idx, target_idx));
    }
    // Construct the twin edges.
    for (int i = 0; i < half_edge_num; ++i) {
        target_half_edge_twins_[i] = GetHalfEdgeIndex(target_half_edges_[i].second, target_half_edges_[i].first);
    }

    // Construct the face not from the polyhedron but from the file (CGAL's data structure is quite hard to understand...).
    target_half_facets_.clear();
    target_half_facet_twins_.clear();
    const int half_facet_num = static_cast<int>(target_.number_of_halffacets());
    target_half_facets_.reserve(half_facet_num);
    target_half_facet_twins_.resize(half_facet_num, -1);

    // Now parse the input file.
    std::ifstream in_file(file_name);
    std::string line;
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
    while (std::getline(in_file, line)) {
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
            line = line.substr(0, line.find(',', line.find(',') + 1));
            std::istringstream iss(line);
            int fid;
            iss >> fid;
            CheckError(fid == line_idx - half_facet_begin, "fid mismatches.");
            char ch;
            iss >> ch;
            iss >> target_half_facet_twins_[fid];
            line = line.substr(line.find(',') + 1);
            const std::vector<std::string> words = SplitString(line);
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
        target_half_facets_.push_back(fc_idx);
    }
}

void Scene::ListAllVertices() {
    std::cout << "Vertex number " << target_vertices_.size() << std::endl;
    int idx = 0;
    for (const auto& v : target_vertices_) {
        const real x = v.x(), y = v.y(), z = v.z();
        std::cout << "v" << idx << "\t" << x << "\t" << y << "\t" << z << std::endl;
        ++idx;
    }
}

void Scene::ListAllEdges() {
    std::cout << "Edge number " << target_half_edges_.size() << std::endl;
    int idx = 0;
    for (const auto& e : target_half_edges_) {
        std::cout << "e" << idx << "\tv" << e.first << "\tv" << e.second << "\ttwin\t"
            << "e" << target_half_edge_twins_[idx] << std::endl;
        ++idx;
    }
}

void Scene::ListAllFaces() {
    std::cout << "Face number " << target_half_facets_.size() << std::endl;
    int idx = 0;
    for (const auto& f : target_half_facets_) {
        std::cout << "f" << idx << "\t" << f.size() << std::endl;
        for (const auto& fc : f) {
            for (const int v : fc) std::cout << "v" << v << "\t";
            std::cout << std::endl;
        }
        std::cout << "twin\t" << "f" << target_half_facet_twins_[idx] << std::endl;
        ++idx;
    }
}

void Scene::Extrude(const std::vector<Vector3r>& polygon, const Vector3r& dir, const char op) {
    // Create the new polyhedron.
    std::stringstream ss;
    const int poly_dof = static_cast<int>(polygon.size());

    // Check the orientation of the polygon.
    std::vector<Vector3r> offset_polygon;
    for (const auto& v : polygon) offset_polygon.push_back(v - polygon[0]);
    real vol = 0;
    for (int i = 0; i < poly_dof; ++i) {
        const int next_i = (i + 1) % poly_dof;
        const Vector3r v0 = offset_polygon[i], v1 = offset_polygon[next_i];
        vol += v0.cross(v1).dot(dir);
    }
    // Do nothing if the extrusion is degenerated.
    if (vol == 0) return;

    const bool reversed = vol < 0;
    // Header.
    ss << "OFF" << std::endl
        << poly_dof * 2 << " " << poly_dof + 2 << " " << poly_dof * 3 << std::endl;
    // Vertices.
    for (const auto& v : polygon) {
        ss << v.x() << " " << v.y() << " " << v.z() << std::endl;
    }
    for (const auto& v : polygon) {
        const Vector3r v2 = v + dir;
        ss << v2.x() << " " << v2.y() << " " << v2.z() << std::endl;
    }
    // Faces.
    if (reversed) {
        ss << poly_dof;
        for (int i = 0; i < poly_dof; ++i) ss << " " << i;
        ss << std::endl << poly_dof;
        for (int i = poly_dof - 1; i >= 0; --i) ss << " " << i + poly_dof;
        ss << std::endl;

        for (int i = 0; i < poly_dof; ++i) {
            const int j = (i + 1) % poly_dof;
            ss << 4 << " " << j << " " << i << " " << i + poly_dof << " " << j + poly_dof << std::endl;
        }
    } else {
        ss << poly_dof;
        for (int i = 0; i < poly_dof; ++i) ss << " " << i + poly_dof;
        ss << std::endl << poly_dof;
        for (int i = poly_dof - 1; i >= 0; --i) ss << " " << i;
        ss << std::endl;

        for (int i = 0; i < poly_dof; ++i) {
            const int j = (i + 1) % poly_dof;
            ss << 4 << " " << i << " " << j << " " << j + poly_dof << " " << i + poly_dof << std::endl;
        }
    }

    Polyhedron poly;
    ss >> poly;
    Nef_polyhedron nef_poly(poly);
    CheckError(nef_poly.is_simple(), "The input is not a 2-manifold.");

    // Boolean operation.
    CheckError(op == '+' || op == '-', "We only support union and difference for now.");
    if (op == '+') {
        objects_ = (objects_ + nef_poly).regularization();
    } else {
        objects_ = (objects_ - nef_poly).regularization();
    }
}

void Scene::ExtrudeFromRef(const int f_idx, const int loop_idx,
    const int v_source, const int v_target, const char op) {
    std::vector<Vector3r> polygon;
    for (const int vid : target_half_facets_[f_idx][loop_idx]) {
        polygon.push_back(target_vertices_[vid]);
    }
    const Vector3r dir = target_vertices_[v_target] - target_vertices_[v_source];
    Extrude(polygon, dir, op);
}

void Scene::SaveScene(const std::string& file_name) {
    // Check the type of the file name.
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u && (name_and_ext[1] == "off" || name_and_ext[1] == "nef3"), "Invalid file name.");
    CheckError(objects_.is_simple(), "The current scene is not a 2-manifold.");
    PrepareToCreateFile(file_name);

    const std::string ext = name_and_ext[1];
    if (ext == "off") {
        Polyhedron poly;
        objects_.convert_to_polyhedron(poly);
        std::ofstream output(file_name);
        write_off(output, poly);
    } else {
        std::ofstream output(file_name);
        output << objects_;
    }
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

const int Scene::GetVertexIndex(const Vector3r& vertex) const {
    int idx = 0;
    for (const auto& v : target_vertices_) {
        if (v.x() == vertex.x() && v.y() == vertex.y() && v.z() == vertex.z()) break;
        ++idx;
    }
    return idx;
}

const int Scene::GetHalfEdgeIndex(const int source, const int target) const {
    int idx = 0;
    for (const auto& e : target_half_edges_) {
        if (e.first == source && e.second == target) break;
        ++idx;
    }
    return idx;
}