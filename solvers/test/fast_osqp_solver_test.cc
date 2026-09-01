#include "solvers/fast_osqp_solver.h"

#include <gtest/gtest.h>

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solver_options.h>

namespace dairlib::solvers {
namespace {

TEST(FastOsqpSolverTest, RepeatedWorkspaceRebuildPreservesSolution) {
  drake::solvers::MathematicalProgram program;
  const auto x = program.NewContinuousVariables<2>("x");
  program.AddQuadraticCost(
      Eigen::Matrix2d::Identity(), Eigen::Vector2d(-2.0, 4.0), x);
  program.AddBoundingBoxConstraint(
      Eigen::Vector2d(-10.0, -10.0), Eigen::Vector2d(10.0, 10.0), x);

  drake::solvers::SolverOptions options;
  FastOsqpSolver solver;
  solver.InitializeSolver(program, options);
  for (int solve = 0; solve < 1000; ++solve) {
    const auto result = solver.Solve(program);
    ASSERT_TRUE(result.is_success());
    EXPECT_TRUE(result.GetSolution(x).isApprox(
        Eigen::Vector2d(2.0, -4.0), 1.0e-5));
  }
}

}  // namespace
}  // namespace dairlib::solvers
