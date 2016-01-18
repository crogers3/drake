#include "ConstantConstraint.h"

namespace drake {
ConstantConstraint::ConstantConstraint(double desired_value,
    int xdim,
    int jAvar) : BoundingBoxConstraint(
        desired_value, desired_value, xdim, jAvar) {}
} // namespace drake