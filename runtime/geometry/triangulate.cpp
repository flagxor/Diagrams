#include "triangulate.h"
#include "geometry2.h"
#include "poly2tri/poly2tri.h"
#include "polyclipping/clipper.hpp"
#include "aabb.h"
#include <stack>
#include <iostream>


namespace {
// maybe we have a template class
inline double dot(const diagrammar::Vec2f& a, diagrammar::Vec2f& b) { return a.adjoint() * b; }

// the dist is between the segment [st, ed] and the point pt
// use double precision to prevent overflow (though not likely to happen)
double SqrdEuclideanDist(const diagrammar::Vec2f& bg, const diagrammar::Vec2f& ed, const diagrammar::Vec2f& pt) {
  // first we check the projection from pt to [st, end];
  diagrammar::Vec2f seg = ed - bg;
  diagrammar::Vec2f pt2bg = pt - bg;
  double projectLength = dot(seg, pt2bg);
  double sqrdSegLength = dot(seg, seg);

  if (projectLength < 0) {
    return dot(pt2bg, pt2bg);
  }

  if (projectLength > sqrdSegLength) {
    diagrammar::Vec2f pt2ed = pt - ed;
    return dot(pt2ed, pt2ed);
  }

  // the segment is degenerate!
  if (sqrdSegLength < 1.e-8) return dot(pt2bg, pt2bg);

  return dot(pt2bg, pt2bg) - projectLength * (projectLength / sqrdSegLength);
}

std::vector<p2t::Point> CleanPolygon(const std::vector<diagrammar::Vec2f>& path) {
  static const float kUScale = 1000.f;
  static const float kDScale = 0.001f;
  ClipperLib::Path scaled_path;
  scaled_path.reserve(path.size());
  for (const auto& pt : path) {
    scaled_path.emplace_back(ClipperLib::IntPoint(pt(0) * kUScale, pt(1) * kUScale));
  }
  ClipperLib::CleanPolygon(scaled_path);
  std::vector<p2t::Point> out;
  out.reserve(scaled_path.size());
  for (const auto& pt : scaled_path) {
    out.emplace_back(pt.X * kDScale, pt.Y * kDScale);
  }
  return out;
}

std::vector<std::vector<p2t::Point> > CleanPolygon(const std::vector<std::vector<diagrammar::Vec2f> >& paths) {
  std::vector<std::vector<p2t::Point> > out;
  out.reserve(paths.size());
  for (const auto& path : paths) {
    out.emplace_back(CleanPolygon(path));
  }
  return out;
}

}

namespace diagrammar {


Triangulation::Triangulation() {}

// Time complexity NlogN like quick sort, worst case is O(N^2)
// this is a recursive algorithm.
void Triangulation::_PolylineDouglasPeuckerRecursive(
    const size_t bg, const size_t ed, const std::vector<Vec2f>& in, float tol,
    std::vector<bool>& out) {
  if (bg == ed - 1) return;
  double maxDist = 0;
  size_t mid = bg + 1;
  for (size_t i = bg + 1; i < ed; ++i) {
    // compute the sqrd distance of every vertices
    double dist = SqrdEuclideanDist(in[bg], in[ed], in[i]);
    if (dist > maxDist) {
      maxDist = dist;
      mid = i;
    }
  }
  // now we have the index of the maxDist point
  if (maxDist > tol * tol) {
    // we will continue in to subgroups
    _PolylineDouglasPeuckerRecursive(bg, mid, in, tol, out);
    _PolylineDouglasPeuckerRecursive(mid, ed, in, tol, out);

  } else {
    // disgard every point between bg and ed
    for (size_t i = bg + 1; i < ed; ++i) {
      out[i] = false;
    }
  }
}

// the iterative version
std::vector<Vec2f> Triangulation::_PolylineDouglasPeuckerIterative(
    const std::vector<Vec2f>& in, float mTol) {
  if (in.size() <= 2) {
    return in;
  }

  std::vector<bool> toKeep(in.size(), true);
  size_t bg = 0;
  size_t ed = in.size() - 1;
  std::stack<std::pair<size_t, size_t> > indexStack;
  indexStack.emplace(std::make_pair(bg, ed));
  // unliked the recursive version, we keep a stack
  while (!indexStack.empty()) {
    const std::pair<size_t, size_t> tmp = indexStack.top();
    indexStack.pop();

    bg = tmp.first;
    ed = tmp.second;
    if (bg == ed - 1) continue;
    double maxDist = 0;
    size_t mid = bg + 1;
    for (size_t i = bg + 1; i < ed; ++i) {
      // compute the sqrd distance of every vertices
      double dist = SqrdEuclideanDist(in[bg], in[ed], in[i]);
      if (dist > maxDist) {
        maxDist = dist;
        mid = i;
      }
    }
    if (maxDist > mTol * mTol) {
      indexStack.emplace(std::make_pair(mid, ed));
      indexStack.emplace(std::make_pair(bg, mid));
    } else {
      // disgard every point between bg and ed
      for (size_t i = bg + 1; i < ed; ++i) {
        toKeep[i] = false;
      }
    }
  }

  std::vector<Vec2f> out;
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    if (toKeep[i]) {
      out.emplace_back(in[i]);
    }
  }
  return out;
}

std::vector<Vec2f> Triangulation::Simplify(const std::vector<Vec2f>& in,
                                           PolylineMethod m) {
  if (in.size() <= 2) {
    return in;
  }

  std::vector<Vec2f> out;
  if (PolylineMethod::kDouglasPeucker == m) {
    AABB bound = GetAABB(in);
    Vec2f span = bound.upper_bound - bound.lower_bound;
    float relTol = 5e-3;
    float tol = std::max(span(0), span(1)) * relTol;
    out = _PolylineDouglasPeuckerIterative(in, tol);
  }
  // std::cout << "polyline simplified: " << in.size() << " : " << out.size()
  //          << std::endl;
  return out;
}

bool isApproxZero(float t) {
  const float tol = 1e-8;
  return fabs(t) < tol;
}

// checking for collinear/degenerate case
int CounterClockwise(const Vec2f& a, const Vec2f& b, const Vec2f& c) {
  Eigen::Vector3f atob(b(0) - a(0), b(1) - a(1), 0);
  Eigen::Vector3f btoc(c(0) - b(0), c(1) - b(1), 0);
  float crossProduct = atob.cross(btoc)(2);
  if (isApproxZero(crossProduct)) return 0;
  if (crossProduct > 0) return 1;
  return -1;
}

// checking for collinear/degenerate case
int PointInCircumcenter(const Vec2f& a, const Vec2f& b, const Vec2f& c,
                        const Vec2f& p) {
  // a, b, c in counter clockwise order!
  if (!CounterClockwise(a, b, c)) exit(-1);

  Eigen::Matrix4f Det;
  Det << a(0), a(1), a.adjoint() * a, 1, b(0), b(1), b.adjoint() * b, 1, c(0),
      c(1), c.adjoint() * c, 1, p(0), p(1), p.adjoint() * p, 1;
  float determinant = Det.determinant();
  if (isApproxZero(determinant)) return 0;
  if (determinant > 0) return 1;
  return -1;
}

std::vector<Triangle2D> Triangulation::DelaunayTriangulation(
    const std::vector<Vec2f>& path) {
  return _DelaunaySweepline(path, nullptr);
}

std::vector<Triangle2D> Triangulation::DelaunayTriangulation(
    const std::vector<Vec2f>& path,
    const std::vector<std::vector<Vec2f> >& holes) {
  return _DelaunaySweepline(path, &holes);
}




std::vector<Triangle2D> Triangulation::InflateAndTriangulate(
    const std::vector<Vec2f>& path) {
  // clipper only works on integer points
  static const float kUScale = 1000.f;
  static const float kDScale = 0.001f;
  ClipperLib::Path in(path.size());

  for (size_t i = 0; i < in.size(); ++i) {
    in[i].X = path[i](0) * kUScale;
    in[i].Y = path[i](1) * kUScale;
  }

  // try to clean it first;
  // now inflate
  ClipperLib::ClipperOffset co;
  co.AddPath(in, ClipperLib::JoinType::jtMiter,
             ClipperLib::EndType::etOpenButt);
  ClipperLib::Paths inflated;
  co.Execute(inflated, 1.5 * kUScale);
  // this is important: prevent p2t from failing
  ClipperLib::CleanPolygons(inflated);
  // one out path
  assert(inflated.size() == 1);
  std::vector<Vec2f> pts(inflated[0].size());
  
  for (size_t i = 0; i < pts.size(); ++i) {
    pts[i](0) = inflated[0][i].X;
    pts[i](1) = inflated[0][i].Y;
  }

  // finally, downscale the points
  std::vector<Triangle2D> out = _DelaunaySweepline(pts, nullptr);
  for (auto& tri : out) {
    tri.p0 = tri.p0 * kDScale;
    tri.p1 = tri.p1 * kDScale;
    tri.p2 = tri.p2 * kDScale;
  }
  return out;
}

std::vector<Triangle2D> Triangulation::_DelaunaySweepline(
    const std::vector<Vec2f>& path,
    const std::vector<std::vector<Vec2f> >* holes) {
  assert(path.size() >= 3);

  // this is important: prevent p2t from failing
  std::vector<p2t::Point> polyline = CleanPolygon(path);
  std::vector<p2t::Point*> polylineptr(polyline.size());
  for (size_t i = 0; i < polyline.size(); ++i) {
    polylineptr[i] = &polyline[i];
  }
  p2t::CDT cdt(polylineptr);

  // similar for the holes
  std::vector<std::vector<p2t::Point> > polyhole;
  if (holes) {
    polyhole = CleanPolygon(*holes);
    for (size_t i = 0; i < polyhole.size(); ++i) {
      std::vector<p2t::Point*> polyholeptr(polyhole[i].size());
      for (size_t j = 0; j < polyhole[i].size(); ++j) {
        polyholeptr[j] = &polyhole[i][j];
      }
      cdt.AddHole(polyholeptr);
    }
  }

  // after we add boundary and holes, begin triangulation
  // NlogN complexity
  cdt.Triangulate();
  std::vector<p2t::Triangle*> triangles;

  // get the list of triangles, the triangle resources
  // are managed by the cdt object
  triangles = cdt.GetTriangles();

  // do a deep copy
  std::vector<Triangle2D> out(triangles.size());
  for (unsigned i = 0; i < triangles.size(); ++i) {
    out[i].p0 =
        Vec2f(triangles[i]->GetPoint(0)->x, triangles[i]->GetPoint(0)->y);
    out[i].p1 =
        Vec2f(triangles[i]->GetPoint(1)->x, triangles[i]->GetPoint(1)->y);
    out[i].p2 =
        Vec2f(triangles[i]->GetPoint(2)->x, triangles[i]->GetPoint(2)->y);
  }

  return out;
}
}