#include <memory>
#include <utility>
#include <vector>

#include "NonlinearProgram.h"
#include "LinearConstraint.h"
#include "QuadraticConstraint.h"
#include "Constraint.h"

namespace snopt {
#include "snopt.hh"
}

using namespace std;
using namespace drake;

#define INF 1.1e+20

void testNonlinearProgram_linearFunction() {
  /**
   * Solves:
   * min    x2
   * st     x1 + x2 = 10
   *        1 <= x1 <= 9
   *
   * Otherwise known as minimum of y = -x + 10, 1 <= x <= 9
   */
  snopt::integer n = 2;
  NonlinearProgram np(n);

  vector<pair<snopt::integer, snopt::doublereal>> A_cost(1, make_pair(2, 1.0));
  unique_ptr<Constraint> cost(new LinearConstraint(-INF, INF, n, A_cost));

  vector<pair<snopt::integer, snopt::doublereal>> A_constr1(2);
  A_constr1[0] = make_pair(1, 1.0);
  A_constr1[1] = make_pair(2, 1.0);
  unique_ptr<Constraint> constr1(new LinearConstraint(10, 10, n, A_constr1));

  vector<pair<snopt::integer, snopt::doublereal>> A_constr2(1, make_pair(1, 1.0));
  unique_ptr<Constraint> constr2(new LinearConstraint(1, 9, n, A_constr2));

  np.setCost(cost);
  np.addConstraint(constr1);
  np.addConstraint(constr2);

  vector<snopt::doublereal> x(n);
  snopt::doublereal objval;
  snopt::integer info;
  np.solve(&x, &objval, &info);

  printf("Value of x: %e\nValue of y: %e\nObjective Value: %e\nInfo: %ld\n", x[0], x[1], objval, info);
}


void testNonlinearProgram_quadraticFunction() {
  /**
   * Solves:
   * min    x2
   * st     x1^2 - 6*x1 - x2 = -9
   *
   * Otherwise known as minimum of y = x^2 -6x + 9
   */
   
  snopt::integer n = 2;
  NonlinearProgram np(n);

  vector<pair<snopt::integer, snopt::doublereal>> A_cost(1, make_pair(2, 1.0));
  unique_ptr<Constraint> cost(new LinearConstraint(-INF, INF, n, A_cost));

  vector<pair<snopt::integer, snopt::doublereal>> Q_constr(1, make_pair(1, 1.0));
  vector<pair<snopt::integer, snopt::doublereal>> b_constr(1, make_pair(1, -6.0));
  vector<pair<snopt::integer, snopt::doublereal>> A_constr(1, make_pair(2, -1.0));
  unique_ptr<Constraint> constr(new QuadraticConstraint(
    -9.0, -9.0, n, Q_constr, b_constr, A_constr));

  np.setCost(cost);
  np.addConstraint(constr);

  vector<snopt::doublereal> x(n);
  snopt::doublereal objval;
  snopt::integer info;
  np.solve(&x, &objval, &info);

  printf("Value of x: %e\nValue of y: %e\nObjective Value: %e\nInfo: %ld\n", x[0], x[1], objval, info);
}

int main() {
  testNonlinearProgram_linearFunction();
  testNonlinearProgram_quadraticFunction();

  return 0;
}
