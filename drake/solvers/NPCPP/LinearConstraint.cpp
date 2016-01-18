#include "LinearConstraint.h"

namespace drake {
LinearConstraint::LinearConstraint(double lb,
    double ub,
    int xdim,
    std::vector<std::pair<int, double>> A_row) : Constraint(lb, ub, xdim) {
  m_A = A_row;
}

void LinearConstraint::nonlinearEval(double x[],
    bool needF,
    bool needG,
    double *f,
    std::vector<double> *g) const {
  // Do Nothing
}
} // namespace drake
