#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <functional>
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

/**
 * Enforces a bounding box constraint: lb <= x[jAvar] <= ub
 * where jAvar in [1, xdim]
 */
class BoundingBoxConstraint : public LinearConstraint {
public:
  BoundingBoxConstraint(double lb,
      double ub,
      int xdim,
      int jAvar);
};

/**
 * Enforces an equality constraint: x[jAvar] = desired_value
 * where jAvar in [1,  xdim]
 */
class ConstantConstraint : public BoundingBoxConstraint {
public:
  ConstantConstraint(double desired_value,
    int xdim,
    int jAvar);
};

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
} // namespace drake

#endif