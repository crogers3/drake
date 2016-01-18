#include "BoundingBoxConstraint.h"

#include <utility>
#include <vector>

namespace drake {
BoundingBoxConstraint::BoundingBoxConstraint(double lb,
    double ub,
    int xdim,
    int jAvar) : LinearConstraint(lb, ub, xdim,
      std::vector<std::pair<int, double>>(1, std::make_pair(jAvar, 1.0))) {}
} // namespace drake