#ifndef LINEAR_CONSTRAINT_H
#define LINEAR_CONSTRAINT_H

#include <utility>
#include <vector>

#include "Constraint.h"

namespace drake {
/**
 * Linear constraint, where: lb <= A_row dot x <= ub
 */
class LinearConstraint : public Constraint {
public:
  LinearConstraint(double lb,
    double ub,
    int xdim,
    std::vector<std::pair<int, double>> A_row);
  void nonlinearEval(double x[],
      bool needF,
      bool needG,
      double *f,
      std::vector<double> *g) const;
};
} // namespace drake

#endif
