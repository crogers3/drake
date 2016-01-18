#ifndef FUNCTION_HANDLE_CONSTRAINT_H
#define FUNCTION_HANDLE_CONSTRAINT_H

#include <functional>
#include <utility>
#include <vector>

#include "Constraint.h"

namespace drake {
/**
 * A constraint implementation where the constraint is given as a function handle
 */
class FunctionHandleConstraint : public Constraint {
protected:
  std::function<void(double[],bool,bool,double*,std::vector<double>*)> m_fun;

public:
  FunctionHandleConstraint(double lb,
      double ub,
      int xdim,
      std::vector<std::pair<int, double>> A_row,
      std::vector<int> jGvar_row,
      std::function<void(double[],bool,bool,double*,std::vector<double>*)> nonlinearEval);
  void nonlinearEval(double x[],
    bool needF,
    bool needG,
    double *f,
    std::vector<double> *g) const;
};
}

#endif