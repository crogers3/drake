#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <utility>
#include <vector>

namespace drake {
/**
 * Abstract constraint class.
 * Subclasses must implement nonlinearEval and must specify:
 *   m_A: a vector containing index, value pairs denoting the linear part of the constraint.
 *        No decision variable may be included in m_A that also has a nonlinear component,
 *        instead, both linear and nonlinear parts must be calculated in nonlinearEval
 *   m_jGvar: a vector containing the indices of decision variables whose gradients are
 *            calculated in nonlinearEval.
 */
class Constraint {
protected:
  double m_lb;
  double m_ub;
  int m_xdim;
  std::vector<std::pair<int, double>> m_A;
  std::vector<int> m_jGvar;

public:
  Constraint(double lb, double ub, int xdim);
  virtual void nonlinearEval(double x[],
      bool needF,
      bool needG,
      double *f,
      std::vector<double> *g) const = 0;
  void getBounds(double* lb, double* ub);
  void getA(int* neA, std::vector<int>* jAvar, std::vector<double>* A) const;
  void getG(int* neG, std::vector<int>* jGvar) const;
};
} // namespace drake

#endif