#ifndef CONSTANT_CONSTRAINT_H
#define CONSTANT_CONSTRAINT_H

#include "BoundingBoxConstraint.h"

namespace drake {
/**
 * Enforces an equality constraint: x[jAvar] = desired_value
 * where jAvar in [1,  xdim]
 */
class ConstantConstraint : public BoundingBoxConstraint {
public:
  ConstantConstraint(double desired_value,
    int xdim,
    int jAvar);
};
} // namespace drake

#endif