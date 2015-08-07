// Copyright 2015 Native Client Authors.

#include <stack>
#include <iostream>
#include <utility>
#include <algorithm>
#include <unordered_map>
#include <vector>

#include "geometry/geometry2.h"
#include "poly2tri/poly2tri.h"
#include "polyclipping/clipper.hpp"
#include "geometry/aabb.h"

namespace {

inline double dot(const diagrammar::Vector2f& a,
                  const diagrammar::Vector2f& b) {
  return a.adjoint() * b;
}

bool IsApproxZero(double t) {
  const double tol = 1e-8;
  return fabs(t) < tol;
}

// the dist is between the segment [st, ed] and the point pt
// use double precision to prevent overflow (though not likely to happen)
double SqrdDistSegmentToPoint(const diagrammar::Vector2f& bg,
                              const diagrammar::Vector2f& ed,
                              const diagrammar::Vector2f& pt) {
  // first we check the projection from pt to [st, end];
  diagrammar::Vector2f seg = ed - bg;
  diagrammar::Vector2f pt2bg = pt - bg;
  double projected_length = dot(seg, pt2bg);
  double squared_seg_length = dot(seg, seg);

  if (projected_length < 0) {
    return dot(pt2bg, pt2bg);
  }

  if (projected_length > squared_seg_length) {
    diagrammar::Vector2f pt2ed = pt - ed;
    return dot(pt2ed, pt2ed);
  }

  // the segment is degenerate (ed is too close to bg)!
  if (IsApproxZero(squared_seg_length)) return dot(pt2bg, pt2bg);

  return dot(pt2bg, pt2bg) -
         projected_length * (projected_length / squared_seg_length);
}

// the clipper library takes only integer points for robustness
// for floating points we have to scale them up (multiply by 1000 to keep at
// most three digits) and convert them into clipper path(s).
// after clipper processing we can then scale down the points;
const float kUScale = 1000.f;
const float kDScale = 0.001f;

ClipperLib::Path UScalePathDiaToClipper(const diagrammar::Path2D& path) {
  ClipperLib::Path scaled_path;
  scaled_path.reserve(path.size());
  for (const auto& pt : path) {
    scaled_path.emplace_back(
        ClipperLib::IntPoint(pt(0) * kUScale, pt(1) * kUScale));
  }
  return scaled_path;
}

std::vector<p2t::Point> DScalePathClipperToPoly(const ClipperLib::Path& path) {
  std::vector<p2t::Point> out;
  out.reserve(path.size());
  for (const auto& pt : path) {
    out.emplace_back(pt.X * kDScale, pt.Y * kDScale);
  }
  return out;
}

diagrammar::Path2D DScalePathClipperToDia(const ClipperLib::Path& path) {
  diagrammar::Path2D out;
  out.reserve(path.size());
  for (const auto& pt : path) {
    out.emplace_back(pt.X * kDScale, pt.Y * kDScale);
  }
  return out;
}

// Wrapper of Clipper CleanPolygon for poly2tri to use
// The function does not check for self intersections!
std::vector<p2t::Point> CleanPolygon(const diagrammar::Path2D& path) {
  ClipperLib::Path scaled_path = UScalePathDiaToClipper(path);
  ClipperLib::CleanPolygon(scaled_path);
  return DScalePathClipperToPoly(scaled_path);
}

std::vector<std::vector<p2t::Point> > CleanPolygons(
    const std::vector<diagrammar::Path2D>& paths) {
  std::vector<std::vector<p2t::Point> > out;
  out.reserve(paths.size());
  for (const auto& path : paths) {
    out.emplace_back(CleanPolygon(path));
  }
  return out;
}

// time complexity NlogN with worst case O(N^2)
// where N is the size of input path
void PolylineDouglasPeuckerRecursive(const size_t bg, const size_t ed,
                                     const diagrammar::Path2D& path, float tol,
                                     std::vector<bool>* out) {
  if (bg == ed - 1) return;
  double max_dist = 0;
  size_t mid = bg + 1;
  for (size_t i = bg + 1; i < ed; ++i) {
    // compute the sqrd distance of every vertices
    double dist = SqrdDistSegmentToPoint(path[bg], path[ed], path[i]);
    if (dist > max_dist) {
      max_dist = dist;
      mid = i;
    }
  }
  if (max_dist > tol * tol) {
    PolylineDouglasPeuckerRecursive(bg, mid, path, tol, out);
    PolylineDouglasPeuckerRecursive(mid, ed, path, tol, out);

  } else {
    // disgard every point between bg and ed
    for (size_t i = bg + 1; i < ed; ++i) {
      (*out)[i] = false;
    }
  }
}

diagrammar::Path2D PolylineDouglasPeuckerIterative(
    const diagrammar::Path2D& path, float tol) {
  if (path.size() <= 2) {
    return path;
  }

  std::vector<bool> toKeep(path.size(), true);
  size_t bg = 0;
  size_t ed = path.size() - 1;
  std::stack<std::pair<size_t, size_t> > indexStack;
  indexStack.emplace(std::make_pair(bg, ed));

  while (!indexStack.empty()) {
    const std::pair<size_t, size_t> tmp = indexStack.top();
    indexStack.pop();

    bg = tmp.first;
    ed = tmp.second;
    if (bg == ed - 1) continue;
    double max_dist = 0;
    size_t mid = bg + 1;
    for (size_t i = bg + 1; i < ed; ++i) {
      double dist = SqrdDistSegmentToPoint(path[bg], path[ed], path[i]);
      if (dist > max_dist) {
        max_dist = dist;
        mid = i;
      }
    }
    if (max_dist > tol * tol) {
      indexStack.emplace(std::make_pair(mid, ed));
      indexStack.emplace(std::make_pair(bg, mid));
    } else {
      // disgard every point between bg and ed
      for (size_t i = bg + 1; i < ed; ++i) {
        toKeep[i] = false;
      }
    }
  }

  diagrammar::Path2D out;
  out.reserve(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    if (toKeep[i]) {
      out.emplace_back(path[i]);
    }
  }
  return out;
}

diagrammar::TriangleMesh2D DelaunaySweepline(
    const diagrammar::Path2D& path,
    const std::vector<diagrammar::Path2D>& holes) {
  assert(path.size() >= 3);
  std::vector<p2t::Point> cleaned_path = CleanPolygon(path);
  std::vector<p2t::Point*> path_ptr(cleaned_path.size());
  for (size_t i = 0; i < cleaned_path.size(); ++i) {
    path_ptr[i] = &cleaned_path[i];
  }
  p2t::CDT cdt(path_ptr);
  std::vector<std::vector<p2t::Point> > polyhole = CleanPolygons(holes);
  for (size_t i = 0; i < polyhole.size(); ++i) {
    std::vector<p2t::Point*> polyholeptr(polyhole[i].size());
    for (size_t j = 0; j < polyhole[i].size(); ++j) {
      polyholeptr[j] = &polyhole[i][j];
    }
    cdt.AddHole(polyholeptr);
  }
  // after we add boundary and holes, begin triangulation
  // NlogN complexity
  cdt.Triangulate();
  std::vector<p2t::Triangle*> triangles;

  // get the list of triangles, the triangle resources
  // are managed by the cdt object
  triangles = cdt.GetTriangles();

  size_t num_vertices = path.size();
  for (auto& hole : polyhole) {
    num_vertices += hole.size();
  }
  std::unordered_map<p2t::Point*, size_t> pt2index;
  diagrammar::TriangleMesh2D mesh;
  mesh.vertices.reserve(num_vertices);
  mesh.faces.resize(triangles.size());
  for (size_t i = 0; i < triangles.size(); ++i) {
    for (size_t vt_idx = 0; vt_idx < 3; ++vt_idx) {
      p2t::Point* vertex = triangles[i]->GetPoint(vt_idx);
      if (pt2index.find(vertex) != pt2index.end()) {
        mesh.faces[i][vt_idx] = pt2index[vertex];
      } else {
        mesh.vertices.emplace_back(vertex->x, vertex->y);
        pt2index[vertex] = mesh.vertices.size() - 1;
        mesh.faces[i][vt_idx] = pt2index[vertex];
      }
    }
  }
  return mesh;
}

// checking for collinear/degenerate case
int CounterClockwise(const diagrammar::Vector2f& a,
                     const diagrammar::Vector2f& b,
                     const diagrammar::Vector2f& c) {
  diagrammar::Vector3f atob(b(0) - a(0), b(1) - a(1), 0);
  diagrammar::Vector3f btoc(c(0) - b(0), c(1) - b(1), 0);
  float crossProduct = atob.cross(btoc)(2);
  if (IsApproxZero(crossProduct)) return 0;
  if (crossProduct > 0) return 1;
  return -1;
}

// checking for collinear/degenerate case
int PointInCircumcenter(const diagrammar::Vector2f& a,
                        const diagrammar::Vector2f& b,
                        const diagrammar::Vector2f& c,
                        const diagrammar::Vector2f& p) {
  // a, b, c in counter clockwise order!
  if (!CounterClockwise(a, b, c)) exit(-1);

  diagrammar::Matrix4f Det;
  Det << a(0), a(1), a.adjoint() * a, 1, b(0), b(1), b.adjoint() * b, 1, c(0),
      c(1), c.adjoint() * c, 1, p(0), p(1), p.adjoint() * p, 1;
  float determinant = Det.determinant();
  if (IsApproxZero(determinant)) return 0;
  if (determinant > 0) return 1;
  return -1;
}

// The method will create a set of strictly simple polygons (described by the
// boundary paths)
// Please check
// http://stackoverflow.com/questions/23578760/how-to-simplify-a-single-complex-uibezierpath-polygon-in-ios
std::vector<diagrammar::Path2D> ResolveIntersectionsClosedPath(
    const diagrammar::Path2D& path) {
  ClipperLib::Path scaled_path = UScalePathDiaToClipper(path);
  ClipperLib::Clipper clipper;
  clipper.AddPath(scaled_path, ClipperLib::ptSubject, true);
  ClipperLib::Paths intermediate_paths;
  clipper.Execute(ClipperLib::ctUnion, intermediate_paths,
                  ClipperLib::pftNonZero, ClipperLib::pftNonZero);
  clipper.Clear();

  for (auto& line : intermediate_paths) {
    if (ClipperLib::Orientation(line)) {
      ClipperLib::ReversePath(line);
    }
  }
  clipper.AddPaths(intermediate_paths, ClipperLib::ptSubject, true);
  ClipperLib::Paths unioned_paths;
  clipper.StrictlySimple(true);
  clipper.Execute(ClipperLib::ctUnion, unioned_paths, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);
  clipper.Clear();
  std::vector<diagrammar::Path2D> resolved_paths;
  for (auto& p : unioned_paths) {
    resolved_paths.emplace_back(DScalePathClipperToDia(p));
  }
  return resolved_paths;
}

// This is a very expensive operation!
std::vector<diagrammar::Path2D> ResolveIntersectionsClosedPaths(
    const std::vector<diagrammar::Path2D>& paths) {
  ClipperLib::Clipper clipper;

  for (auto& path : paths) {
    ClipperLib::Path scaled_path = UScalePathDiaToClipper(path);
    clipper.AddPath(scaled_path, ClipperLib::ptSubject, true);
  }
  ClipperLib::Paths intermediate_paths;
  clipper.Execute(ClipperLib::ctUnion, intermediate_paths,
                  ClipperLib::pftNonZero, ClipperLib::pftNonZero);
  clipper.Clear();

  for (auto& line : intermediate_paths) {
    if (ClipperLib::Orientation(line)) {
      ClipperLib::ReversePath(line);
    }
  }
  clipper.AddPaths(intermediate_paths, ClipperLib::ptSubject, true);
  ClipperLib::Paths unioned_paths;
  clipper.StrictlySimple(true);
  clipper.Execute(ClipperLib::ctUnion, unioned_paths, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);
  clipper.Clear();

  std::vector<diagrammar::Path2D> resolved_paths;
  for (auto& p : unioned_paths) {
    resolved_paths.emplace_back(DScalePathClipperToDia(p));
  }

  return resolved_paths;
}

// Input must be three points from a ccw oriented curve (if it is closed)!
diagrammar::Vector2f GetNormalFromTwoPoints(const diagrammar::Vector2f& p0,
                                            const diagrammar::Vector2f& p1) {
  diagrammar::Matrix2f cw_90;
  cw_90 << 0, 1, -1, 0;
  diagrammar::Vector2f normal_vec(cw_90 * (p1 - p0));
  return normal_vec.normalized();
}

}  // namespace

namespace diagrammar {
CollisionShape2D::~CollisionShape2D() {}

Path2D SketchDiskEdge(const Disk2D& disk, size_t num_vertices) {
  Path2D boundary(num_vertices);
  for (size_t i = 0; i < num_vertices; ++i) {
    float theta = 2 * i * M_PI / static_cast<float>(num_vertices);
    boundary[i] = Vector2f(disk.radius * cos(theta), disk.radius * sin(theta));
  }
  return boundary;
}

std::vector<Vector2f> SimplifyPolyline(const Path2D& path, float rel_tol) {
  if (path.size() <= 2) {
    return path;
  }

  std::vector<Vector2f> out;
  if (true) {
    // we compute the boundary square size of the path
    AABB bound = GetAABBWithPadding(path, 0);
    Vector2f span = bound.upper_bound - bound.lower_bound;
    // set a tolerance to remove points that are too close to
    // each other. the relative tolerance is 0.5% of max size
    // so the simplified curve is still pretty smooth
    float tol = std::max(span(0), span(1)) * rel_tol;
    out = PolylineDouglasPeuckerIterative(path, tol);
  } else {
    // only one method implemented
    // the douglas_peucker method may change the topology of input curves
    assert(0);
  }
  return out;
}

TriangleMesh2D TriangulatePolygon(const Polygon2D& polygon) {
  return DelaunaySweepline(polygon.path, polygon.holes);
}

TriangleMesh2D TriangulatePolyline(const Path2D& path, float offset) {
  // clipper only works with integer points
  ClipperLib::Path scaled_path = UScalePathDiaToClipper(path);

  // inflate
  ClipperLib::ClipperOffset co;
  co.AddPath(scaled_path, ClipperLib::JoinType::jtMiter,
             ClipperLib::EndType::etOpenButt);
  ClipperLib::Paths inflated;
  // inflate by input amount.
  co.Execute(inflated, offset * kUScale);

  // one out path
  // for now, we do not allow the holes to appear in the inflated path
  // maybe we allow this later using union
  // assert(inflated.size() == 1 && "inflated path has holes");
  std::vector<Vector2f> pts = DScalePathClipperToDia(inflated[0]);
  return DelaunaySweepline(pts, std::vector<Path2D>());
}
/*
std::vector<Polygon> DecomposePolygonToConvexhulls(const Polygon& polygon) {
  using VHACD::IVHACD;

  TriangleMesh mesh = TriangulatePolygon(polygon);
  std::vector<float> points;
  points.reserve(2 * mesh.vertices.size());
  for (auto& vertex : mesh.vertices) {
    points.emplace_back(vertex(0));
    points.emplace_back(vertex(1));
  }

  std::vector<int> triangle_indices;
  triangle_indices.reserve(mesh.faces.size() * 3);
  for (auto& tr_idx : mesh.faces) {
    triangle_indices.emplace_back(tr_idx[0]);
    triangle_indices.emplace_back(tr_idx[1]);
    triangle_indices.emplace_back(tr_idx[2]);
  }

  IVHACD::Parameters params;
  //
  // params.m_maxNumVerticesPerCH = 8;
  params.m_oclAcceleration = false;
  IVHACD* vhacd_interface = VHACD::CreateVHACD();
  bool res = vhacd_interface->Compute(points.data(), 2, mesh.vertices.size(),
                                      triangle_indices.data(), 3,
                                      mesh.faces.size(), params);
  std::vector<Polygon> polygons;
  if (res) {
    size_t num_hulls = vhacd_interface->GetNConvexHulls();
    IVHACD::ConvexHull hull;
    for (size_t p = 0; p < num_hulls; ++p) {
      vhacd_interface->GetConvexHull(p, hull);
      for (size_t v = 0; v < hull.m_nPoints; ++v) {
        std::cout << p << " ";
        std::cout << hull.m_points[3 * v + 0] << " ";
        std::cout << hull.m_points[3 * v + 1] << " ";
        std::cout << hull.m_points[3 * v + 2] << "\n";
      }
    }
  } else {
    std::cerr << "convex hull decomposition not successfull! fall back to "
                 "triangulation!\n";
  }

  vhacd_interface->Clean();
  vhacd_interface->Release();
  exit(0);
  return polygons;
}
*/
std::vector<Polygon2D> ResolveIntersections(const Polygon2D& polygon) {
  // the polygon boundary maybe splitted during this process
  // auto paths = ResolveIntersectionsClosedPath(polygon.path);
  // auto holes = ResolveIntersectionsClosedPaths(polygon.holes);

  ClipperLib::Clipper clipper;
  ClipperLib::Path scaled_path = UScalePathDiaToClipper(polygon.path);
  clipper.AddPath(scaled_path, ClipperLib::ptSubject, true);

  /*
  for (auto& path : paths) {
    ClipperLib::Path scaled_path = UScalePathDiaToClipper(path);
    clipper.AddPath(scaled_path, ClipperLib::ptSubject, true);
  }*/

  for (auto& hole : polygon.holes) {
    ClipperLib::Path scaled_hole = UScalePathDiaToClipper(hole);
    clipper.AddPath(scaled_hole, ClipperLib::ptClip, true);
  }

  ClipperLib::PolyTree path_tree;
  clipper.StrictlySimple(true);
  clipper.Execute(ClipperLib::ctDifference, path_tree, ClipperLib::pftNonZero,
                  ClipperLib::pftNonZero);

  // iterating into the tree
  std::vector<Polygon2D> polygons;
  // only store the pointer to outer polygons
  std::unordered_map<ClipperLib::PolyNode*, size_t> polynode_map;
  for (ClipperLib::PolyNode* node_ptr = path_tree.GetFirst(); node_ptr;
       node_ptr = node_ptr->GetNext()) {
    ClipperLib::PolyNode* poly_ptr = node_ptr;
    while (poly_ptr && poly_ptr->IsHole()) {
      poly_ptr = poly_ptr->Parent;
    }
    if (polynode_map.find(poly_ptr) == polynode_map.end()) {
      polygons.emplace_back(Polygon2D());
      polygons.back().path = DScalePathClipperToDia(poly_ptr->Contour);
      polynode_map[poly_ptr] = polygons.size() - 1;
    } else {
      polygons[polynode_map[poly_ptr]].holes.emplace_back(
          DScalePathClipperToDia(node_ptr->Contour));
    }
  }
  return polygons;
}

std::vector<Vector2f> GeneratePathNormals(const Path2D& path, bool is_closed,
                                          bool outward) {
  assert(path.size() >= 2);
  // Here we are going to use a simple 3-point interpolation method
  // first we need to make the path in ccw direction
  ClipperLib::Path scaled_path = UScalePathDiaToClipper(path);
  if (!ClipperLib::Orientation(scaled_path)) {
    ClipperLib::ReversePath(scaled_path);
  }
  Path2D ccw_path = DScalePathClipperToDia(scaled_path);

  // Special treatment for the first and last point.
  std::vector<Vector2f> output_vecs(path.size());
  if (is_closed) {
    output_vecs[0] =
        GetNormalFromTwoPoints(ccw_path[ccw_path.size() - 1], ccw_path[1]);
    output_vecs.back() =
        GetNormalFromTwoPoints(ccw_path[ccw_path.size() - 2], ccw_path[0]);
  } else {
    output_vecs[0] = GetNormalFromTwoPoints(ccw_path[0], ccw_path[1]);
    output_vecs.back() = GetNormalFromTwoPoints(ccw_path[ccw_path.size() - 2],
                                                ccw_path[ccw_path.size() - 1]);
  }

  // We use the point before and after ith point to interpolate its normal.
  for (size_t i = 1; i < ccw_path.size() - 1; ++i) {
    output_vecs[i] = GetNormalFromTwoPoints(ccw_path[i - 1], ccw_path[i + 1]);
  }

  // Reverse normal direction
  if (!outward) {
    for (size_t i = 0; i < ccw_path.size(); ++i) {
      output_vecs[i] = -output_vecs[i];
    }
  }
  return output_vecs;
}

}  // namespace diagrammar
