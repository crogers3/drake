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
} // namespace drake