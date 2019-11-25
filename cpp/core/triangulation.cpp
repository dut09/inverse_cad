#include "core/triangulation.h"
#include "core/common.h"

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

const Vector_3 GetPolygonNormal(const std::vector<Point_3>& polygon) {
    // Project the facet to a 2d polygon.
    // Build the local frame.
    const Point_3 v0 = polygon[0];
    const Point_3 v1 = polygon[1];
    const Vector_3 x = v1 - v0;
    Vector_3 z(0, 0, 0);
    const int n = static_cast<int>(polygon.size());
    for (int vid = 2; vid < n; ++vid) {
        const Point_3& v2 = polygon[vid];
        if (!CGAL::collinear(v0, v1, v2)) {
            z = CGAL::cross_product(x, v2 - v1);
            break;
        }
    }
    // In case we need error message.
    std::ostringstream oss;
    oss << "Polyhedron is degenerated. ";
    for (const auto& p : polygon) oss << p << " ";
    CheckError(z.squared_length() > 0, oss.str());
    return z;
}

CDT Triangulate(const std::vector<Point_3>& polygon, const std::vector<int>& vertex_cycle) {
    CheckError(polygon.size() == vertex_cycle.size(), "Inconsistent polygon and vertex_cycle size.");
    const Point_3 v0 = polygon[0];
    Vector_3 x = polygon[1] - v0;
    const Vector_3 z = GetPolygonNormal(polygon);
    Vector_3 y = CGAL::cross_product(x, z);
    x /= std::sqrt(CGAL::to_double(x.squared_length()));
    y /= std::sqrt(CGAL::to_double(y.squared_length()));

    // Add points and constraints.
    CDT cdt;
    std::vector<CDT::Point> points;
    const int n = static_cast<int>(polygon.size());
    for (int i = 0; i < n; ++i) {
        const Point_3 v = polygon[i];
        const CDT::Point p((v - v0) * x, (v - v0) * y);
        points.push_back(p);
        cdt.insert(p)->id() = vertex_cycle[i];
    }
    cdt.insert_constraint(points.begin(), points.end(), true);

    // Mark facets that are inside the domain bounded by the polygon.
    MarkDomains(cdt);

    return cdt;
}