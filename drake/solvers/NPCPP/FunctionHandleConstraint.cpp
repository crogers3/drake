#include "FunctionHandleConstraint.h"

namespace drake {
FunctionHandleConstraint::FunctionHandleConstraint(double lb,
    double ub,
    int xdim,
    std::vector<std::pair<int, double>> A_row,
    std::vector<int> jGvar_row,
    std::function<void(double[],bool,bool,double*,std::vector<double>*)> nonlinearEval) : Constraint(lb, ub, xdim) {
  m_A = A_row;
  m_jGvar = jGvar_row;
  m_fun = nonlinearEval;
}

void FunctionHandleConstraint::nonlinearEval(double x[],
    bool needF,
    bool needG,
    double *f,
    std::vector<double> *g) const {
  m_fun(x, needF, needG, f, g);
}

}