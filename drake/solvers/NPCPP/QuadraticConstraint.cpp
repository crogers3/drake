#include "QuadraticConstraint.h"

namespace drake {
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

} // namespace drake
