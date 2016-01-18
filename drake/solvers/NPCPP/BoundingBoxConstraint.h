#ifndef BOUNDING_BOX_CONSTRAINT_H
#define BOUNDING_BOX_CONSTRAINT_H

#include "LinearConstraint.h"

namespace drake {
/**
 * Enforces a bounding box constraint: lb <= x[jAvar] <= ub
 * where jAvar in [1, xdim]
 */
class BoundingBoxConstraint : public LinearConstraint {
public:
  BoundingBoxConstraint(double lb,
      double ub,
      int xdim,
      int jAvar);
};
} // namespace drake

#endif