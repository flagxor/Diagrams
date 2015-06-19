#ifndef _DIAGRAMMAR_AABB_
#define _DIAGRAMMAR_AABB_

#include <Eigen/Dense>
#include <vector>

namespace diagrammar {
typedef struct AABB {
  Eigen::Vector2f lower_bound;
  Eigen::Vector2f upper_bound;
} AABB;

static AABB GetAABB(const std::vector<Eigen::Vector2f>& closure) {
  float xmin = closure[0](0);
  float xmax = closure[0](0);
  float ymin = closure[0](1);
  float ymax = closure[0](1);
  for (const auto& pt : closure) {
    if (pt(0) < xmin) xmin = pt(0);
    if (pt(0) > xmax) xmax = pt(0);
    if (pt(1) < ymin) ymin = pt(1);
    if (pt(1) > ymax) ymax = pt(1);
  }

  AABB bound;
  Eigen::Vector2f padding(0.05 * (xmax - xmin), 0.05 * (ymax - ymin));
  bound.lower_bound(0) = xmin;
  bound.lower_bound(1) = ymin;
  bound.lower_bound -= padding;
  bound.upper_bound(0) = xmax;
  bound.upper_bound(1) = ymax;
  bound.upper_bound += padding;
  return bound;
}
}
#endif