#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/snopt_solver.h>
#include <test_vars_constr_cost.h>


using namespace ifopt;

int main()
{
  Problem nlp;

  nlp.AddVariableSet  (std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  nlp.AddCostSet      (std::make_shared<ExCost>());
  nlp.PrintCurrent();

  SnoptSolver solver;
  solver.Solve(nlp);

  std::cout << nlp.GetOptVariables()->GetValues().transpose() << std::endl;
}

