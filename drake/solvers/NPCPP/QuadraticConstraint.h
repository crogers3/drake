#ifndef QUADRATIC_CONSTRAINT_H
#define QUADRATIC_CONSTRAINT_H

#include <utility>
#include <vector>

#include "Constraint.h"

namespace drake {
/**
 * A quadratic constraint, where: lb <= .5 * Q * x'*eye(xdim)*x + b*x <= ub
 * where:
 *   Q: a vector of index, value pairs (sparse 1 x xdim vector)
 *   b: a vector of index, value pairs (sparse 1 x xdim vector)
 */
class QuadraticConstraint : public Constraint {
private:
  void findIfVarInQAndB(std::vector<bool>* varInQ, std::vector<bool>* varInB) const;
  void findVarQAndBVals(std::vector<double>* varQval, std::vector<double>* varBval) const;
protected:
  std::vector<std::pair<int, double>> m_Q;
  std::vector<std::pair<int, double>> m_b;

public:
  QuadraticConstraint(double lb,
      double ub,
      int xdim,
      std::vector<std::pair<int, double>> Q_row,
      std::vector<std::pair<int, double>> b_row,
      std::vector<std::pair<int, double>> A_row);
  void nonlinearEval(double x[],
      bool needF,
      bool needG,
      double *f,
      std::vector<double> *g) const;
};
} // namespace drake

#endif