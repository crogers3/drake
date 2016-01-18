#include "Constraint.h"

namespace drake {
Constraint::Constraint(double lb,
    double ub, int xdim) : m_lb(lb), m_ub(ub), m_xdim(xdim) {}

void Constraint::getBounds(double* lb, double* ub) {
  *lb = m_lb;
  *ub = m_ub;
}

void Constraint::getA(int* neA,
    std::vector<int>* jAvar,
    std::vector<double>* A) const {
  *neA = m_A.size();
  jAvar->clear();
  A->clear();
  for (int i = 0; i < *neA; ++i)  {
    std::pair<int, double> pair = m_A[i];
    jAvar->push_back(pair.first);
    A->push_back(pair.second);
  }
}

void Constraint::getG(int* neG,
    std::vector<int>* jGvar) const {
  *neG = m_jGvar.size();
  jGvar->clear();
  for (int i = 0; i < *neG; ++i)  {
    jGvar->push_back(m_jGvar[i]);
  }
}

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

BoundingBoxConstraint::BoundingBoxConstraint(double lb,
    double ub,
    int xdim,
    int jAvar) : LinearConstraint(lb, ub, xdim,
      std::vector<std::pair<int, double>>(1, std::make_pair(jAvar, 1.0))) {}

ConstantConstraint::ConstantConstraint(double desired_value,
    int xdim,
    int jAvar) : BoundingBoxConstraint(
        desired_value, desired_value, xdim, jAvar) {}

QuadraticConstraint::QuadraticConstraint(double lb,
    double ub,
    int xdim,
    std::vector<std::pair<int, double>> Q_row,
    std::vector<std::pair<int, double>> b_row,
    std::vector<std::pair<int, double>> A_row) : Constraint(lb, ub, xdim) {
  m_Q = Q_row;
  m_b = b_row;
  m_A = A_row;

  // Generate G
  std::vector<bool> varInQ(m_xdim, false);
  std::vector<double> varQval(m_xdim, 0.0);
  std::vector<bool> varInB(m_xdim, false);
  std::vector<double> varBval(m_xdim, 0.0);
  findIfVarInQAndB(&varInQ, &varInB);
  findVarQAndBVals(&varQval, &varBval);
  m_jGvar.clear();
  for (int i = 0; i < m_xdim; ++i) {
    if (varInQ[i] || varInB[i]) {
      m_jGvar.push_back(i + 1);
    }
  }
}

void QuadraticConstraint::nonlinearEval(double x[],
    bool needF,
    bool needG,
    double *f,
    std::vector<double> *g) const {
  std::vector<bool> varInQ(m_xdim, false);
  std::vector<double> varQval(m_xdim, 0.0);
  std::vector<bool> varInB(m_xdim, false);
  std::vector<double> varBval(m_xdim, 0.0);
  findIfVarInQAndB(&varInQ, &varInB);
  findVarQAndBVals(&varQval, &varBval);
  if (needF) {
    *f = 0;
    for (int i = 0; i < m_xdim; ++i) {
      if (varInQ[i]) {
        *f = *f + (varQval[i] * x[i] * x[i]);
      }
      if (varInB[i]) {
        *f = *f + (varBval[i] * x[i]);
      }
    }
  }
  if (needG) {
    g->clear();
    for (int i = 0; i < m_xdim; ++i) {
      if (varInQ[i] || varInB[i]) {
        double thisG = 0;
        if (varInQ[i]) {
          thisG = thisG + (2 * varQval[i] * x[i]);
        }
        if (varInB[i]) {
          thisG = thisG + (varBval[i]);
        }
        g->push_back(thisG);
      }
    }
  }
}

void QuadraticConstraint::findIfVarInQAndB(std::vector<bool>* varInQ, std::vector<bool>* varInB) const {
  int neQ = m_Q.size();
  for (int i = 0; i < neQ; ++i) {
    int var_index = m_Q[i].first - 1;
    (*varInQ)[var_index] = true;
  }
  int neb = m_b.size();
  for (int i = 0; i < neb; ++i) {
    int var_index = m_b[i].first - 1;
    (*varInB)[var_index] = true;
  }
}

void QuadraticConstraint::findVarQAndBVals(std::vector<double>* varQval, std::vector<double>* varBval) const {
  int neQ = m_Q.size();
  for (int i = 0; i < neQ; ++i) {
    int var_index = m_Q[i].first - 1;
    (*varQval)[var_index] = m_Q[i].second;
  }
  int neb = m_b.size();
  for (int i = 0; i < neb; ++i) {
    int var_index = m_b[i].first - 1;
    (*varBval)[var_index] = m_b[i].second;
  }
}

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
} // namespace drake