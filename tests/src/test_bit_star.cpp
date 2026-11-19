/*
Copyright (c) 2026, Manuel Beschi, JRL-CARI CNR-STIIMA/UNIBS, manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <bit_star/solvers/bit_star.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <cnr_class_loader/multi_library_class_loader.hpp>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>
#include <graph_core/samplers/informed_sampler.h>
#include <graph_core/solvers/rrt_star.h>

namespace
{
using graph::bit_star::BITStar;
using graph::bit_star::BITStarPtr;
using graph::core::CollisionCheckerPtr;
using graph::core::NodePtr;
using graph::core::PathPtr;

/// Every configuration is collision-free.
class FreeSpaceChecker : public graph::core::CollisionCheckerBase
{
public:
  explicit FreeSpaceChecker(const graph::core::LoggerPtr& logger) : CollisionCheckerBase(logger, 0.01)
  {
  }
  bool check(const Eigen::VectorXd& configuration) override
  {
    (void)configuration;
    return true;
  }
  CollisionCheckerPtr clone() override
  {
    return std::make_shared<FreeSpaceChecker>(logger_);
  }
};

/// Planar wall |x| <= half_thickness with a slit |y| < half_gap: a narrow passage.
class WallChecker : public graph::core::CollisionCheckerBase
{
public:
  static constexpr double half_thickness = 0.05;
  static constexpr double half_gap = 0.1;

  explicit WallChecker(const graph::core::LoggerPtr& logger) : CollisionCheckerBase(logger, 0.005)
  {
  }
  bool check(const Eigen::VectorXd& q) override
  {
    return std::abs(q(0)) > half_thickness || std::abs(q(1)) < half_gap;
  }
  CollisionCheckerPtr clone() override
  {
    return std::make_shared<WallChecker>(logger_);
  }
};

/// Planning problem: bounds, start, goal and the graph_core components.
struct Problem
{
  graph::core::LoggerPtr logger;
  CollisionCheckerPtr checker;
  graph::core::MetricsPtr metrics;
  graph::core::SamplerPtr sampler;
  NodePtr start;
  NodePtr goal;
};

Problem makeProblem(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double& bound,
                    const std::function<CollisionCheckerPtr(const graph::core::LoggerPtr&)>& make_checker)
{
  Problem p;
  p.logger = std::make_shared<graph::core::ConsoleLogger>("test_bit_star", graph::core::LogLevel::Error);
  p.checker = make_checker(p.logger);
  p.metrics = std::make_shared<graph::core::EuclideanMetrics>(p.logger);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(start.size(), -bound);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(start.size(), bound);
  p.sampler = std::make_shared<graph::core::InformedSampler>(start, goal, lb, ub, p.logger);
  p.start = std::make_shared<graph::core::Node>(start, p.logger);
  p.goal = std::make_shared<graph::core::Node>(goal, p.logger);
  return p;
}

graph::core::ParamMap defaultParams()
{
  graph::core::ParamMap params;
  params["samples_per_batch"] = int64_t(100);
  params["rewire_factor"] = 1.1;
  params["knearest"] = false;
  params["prune"] = true;
  params["prune_threshold"] = 0.05;
  params["utopia_tolerance"] = 0.001;
  params["use_kdtree"] = true;
  params["max_distance"] = 1.0;
  params["extend"] = false;
  params["sample_attempts_factor"] = int64_t(10);
  return params;
}

Problem wallProblem()
{
  Eigen::VectorXd start(2), goal(2);
  start << -1.0, 0.8;
  goal << 1.0, 0.8;
  return makeProblem(start, goal, 1.5,
                     [](const graph::core::LoggerPtr& l) { return std::make_shared<WallChecker>(l); });
}

/// Optimal cost of wallProblem(): through the corners (-t, g) and (t, g) of the slit.
double wallOptimalCost()
{
  const double t = WallChecker::half_thickness;
  const double g = WallChecker::half_gap;
  return 2.0 * std::hypot(1.0 - t, 0.8 - g) + 2.0 * t;
}

Problem cubeProblem()
{
  Eigen::VectorXd start(3), goal(3);
  start << -1.5, -1.5, -1.5;
  goal << 1.5, 1.5, 1.5;
  return makeProblem(start, goal, 2.5, [](const graph::core::LoggerPtr& l) {
    return std::make_shared<graph::core::Cube3dCollisionChecker>(l, 1.0, 0.01);
  });
}

/// Cost of the best path of cubeProblem() that bends once on an edge of the cube: through (-1, 1, 0), sqrt(35).
double cubeOneBendCost()
{
  return std::sqrt(35.0);
}

/// Checks that the path goes from start to goal through collision-free, consistent connections.
void expectValidPath(const PathPtr& path, const Problem& p)
{
  ASSERT_TRUE(path);
  std::vector<graph::core::ConnectionPtr> connections = path->getConnections();
  ASSERT_FALSE(connections.empty());
  EXPECT_EQ(connections.front()->getParent(), p.start);
  EXPECT_EQ(connections.back()->getChild(), p.goal);

  double cost = 0.0;
  for (size_t i = 0; i < connections.size(); i++)
  {
    const graph::core::ConnectionPtr& conn = connections[i];
    if (i > 0)
    {
      EXPECT_EQ(conn->getParent(), connections[i - 1]->getChild());
    }
    EXPECT_TRUE(
        p.checker->checkConnection(conn->getParent()->getConfiguration(), conn->getChild()->getConfiguration()));
    EXPECT_NEAR(conn->getCost(), p.metrics->cost(conn->getParent(), conn->getChild()), 1e-9);
    cost += conn->getCost();
  }
  EXPECT_NEAR(cost, path->cost(), 1e-9);
}

BITStarPtr makeSolver(const Problem& p)
{
  return std::make_shared<BITStar>(p.metrics, p.checker, p.sampler, p.logger);
}

class BITStarTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::srand(42);  // Eigen::Random
  }
};

TEST_F(BITStarTest, FreeSpaceFindsStraightLine)
{
  Eigen::VectorXd start(3), goal(3);
  start << -1.0, -1.0, -1.0;
  goal << 1.0, 1.0, 1.0;
  Problem p = makeProblem(start, goal, 2.0,
                          [](const graph::core::LoggerPtr& l) { return std::make_shared<FreeSpaceChecker>(l); });
  BITStarPtr solver = makeSolver(p);

  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, defaultParams(), solution, 1.0, 100000));
  expectValidPath(solution, p);

  // direct_connection (default true) finds the straight line when the problem is set, before any batch
  EXPECT_EQ(solution->getConnections().size(), 1u);
  EXPECT_NEAR(solution->cost(), (goal - start).norm(), 1e-9);
  EXPECT_FALSE(solver->canImprove());
  EXPECT_EQ(solver->getNumberOfBatches(), 0u);
}

TEST_F(BITStarTest, FreeSpaceWithoutDirectConnectionConverges)
{
  Eigen::VectorXd start(3), goal(3);
  start << -1.0, -1.0, -1.0;
  goal << 1.0, 1.0, 1.0;
  Problem p = makeProblem(start, goal, 2.0,
                          [](const graph::core::LoggerPtr& l) { return std::make_shared<FreeSpaceChecker>(l); });
  BITStarPtr solver = makeSolver(p);
  graph::core::ParamMap params = defaultParams();
  params["direct_connection"] = false;

  // The goal is farther than the RGG radius: the path is a polyline that tends to the straight line
  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, params, solution, 1.0, 100000000));
  expectValidPath(solution, p);
  EXPECT_GT(solution->getConnections().size(), 1u);
  EXPECT_GE(solution->cost(), (goal - start).norm() - 1e-9);
  EXPECT_LE(solution->cost(), 1.03 * (goal - start).norm());
}

TEST_F(BITStarTest, WallWithSlitFindsNearOptimalPath)
{
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);

  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, defaultParams(), solution, 2.0, 100000000));
  expectValidPath(solution, p);

  EXPECT_GE(solution->cost(), wallOptimalCost() - 1e-9);
  EXPECT_LE(solution->cost(), 1.10 * wallOptimalCost());
  EXPECT_GT(solver->getNumberOfBatches(), 1u);
}

TEST_F(BITStarTest, CubeObstacle3DFindsValidPath)
{
  Problem p = cubeProblem();
  BITStarPtr solver = makeSolver(p);

  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, defaultParams(), solution, 1.0, 100000000));
  expectValidPath(solution, p);
  EXPECT_GT(solution->cost(), (p.goal->getConfiguration() - p.start->getConfiguration()).norm());
  EXPECT_LE(solution->cost(), 1.05 * cubeOneBendCost());
}

TEST_F(BITStarTest, BeatsRRTStarOnCubeObstacle)
{
  // Same problem, sampler type, parameters and time budget for both solvers; median over a few runs, since the
  // samplers are random. Measured on a desktop (median of 7 runs): at 0.25 s BIT* ~5.99, RRT* ~6.08; BIT* reaches
  // after 0.1 s a cost that RRT* does not reach in 1 s, so the check holds also in Debug or sanitizer builds.
  const unsigned int runs = 5;
  const double max_time = 0.25;

  auto median = [](std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  };
  auto cost_after = [&](const bool use_bit_star) {
    Problem p = cubeProblem();
    graph::core::TreeSolverPtr solver;
    if (use_bit_star)
      solver = makeSolver(p);
    else
      solver = std::make_shared<graph::core::RRTStar>(p.metrics, p.checker, p.sampler, p.logger);
    PathPtr solution;
    solver->computePath(p.start, p.goal, defaultParams(), solution, max_time, 100000000);
    if (solution)
      expectValidPath(solution, p);
    return solver->solved() ? solver->getCost() : std::numeric_limits<double>::infinity();
  };

  std::vector<double> bit_star_costs, rrt_star_costs;
  for (unsigned int run = 0; run < runs; run++)
  {
    bit_star_costs.push_back(cost_after(true));
    rrt_star_costs.push_back(cost_after(false));
  }
  const double bit_star = median(bit_star_costs);
  const double rrt_star = median(rrt_star_costs);

  std::cout << "Cube 3D, median cost after " << max_time << " s over " << runs << " runs: BIT* " << bit_star
            << ", RRT* " << rrt_star << " (one-bend optimum " << cubeOneBendCost() << ")" << std::endl;
  RecordProperty("bit_star_median_cost", std::to_string(bit_star));
  RecordProperty("rrt_star_median_cost", std::to_string(rrt_star));

  EXPECT_LT(bit_star, rrt_star);
}

TEST_F(BITStarTest, KNearestFindsValidPath)
{
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);
  graph::core::ParamMap params = defaultParams();
  params["knearest"] = true;

  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, params, solution, 2.0, 100000000));
  expectValidPath(solution, p);
  EXPECT_GT(solver->getKNeighbours(), 0u);
  EXPECT_LE(solution->cost(), 1.20 * wallOptimalCost());
}

TEST_F(BITStarTest, CostNeverIncreases)
{
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);
  ASSERT_TRUE(solver->config(defaultParams()));
  ASSERT_TRUE(solver->addStart(p.start));
  ASSERT_TRUE(solver->addGoal(p.goal));

  std::vector<double> costs;
  PathPtr solution;
  for (unsigned int iter = 0; iter < 200000 && solver->canImprove(); iter++)
  {
    if (solver->update(solution) && solution)
      costs.push_back(solver->getCost());
  }

  ASSERT_GE(costs.size(), 2u);
  for (size_t i = 1; i < costs.size(); i++)
    EXPECT_LE(costs[i], costs[i - 1]);
  EXPECT_LT(costs.back(), costs.front());
}

TEST_F(BITStarTest, CachedCostToComeMatchesTree)
{
  // Long enough to go through several prunings and rewirings
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);
  graph::core::ParamMap params = defaultParams();
  params["prune_threshold"] = 0.0;

  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, params, solution, 1.0, 100000000));
  EXPECT_GT(solver->getNumberOfRewirings(), 0u);

  graph::core::TreePtr tree = solver->getStartTree();
  for (const NodePtr& node : tree->getNodes())
  {
    ASSERT_TRUE(node == p.start || node->getParentConnectionsSize() == 1);
    EXPECT_NEAR(solver->getCostToCome(node), tree->costToNode(node), 1e-9);
  }
  EXPECT_NEAR(solver->getCostToCome(p.goal), solver->getPathCost(), 1e-9);
}

TEST_F(BITStarTest, PruningKeepsOnlyPromisingVertices)
{
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);
  graph::core::ParamMap params = defaultParams();
  params["prune_threshold"] = 0.0;
  ASSERT_TRUE(solver->config(params));
  ASSERT_TRUE(solver->addStart(p.start));
  ASSERT_TRUE(solver->addGoal(p.goal));

  // With prune_threshold = 0, every batch that starts after an improvement prunes with the current cost: right
  // after it, no vertex can have f̂ above the cost of the solution.
  PathPtr solution;
  unsigned int batches = 0;
  unsigned int checked_prunings = 0;
  double cost_at_last_batch = std::numeric_limits<double>::infinity();
  for (unsigned int iter = 0; iter < 2000000 && checked_prunings < 5 && solver->canImprove(); iter++)
  {
    solver->update(solution);
    if (solver->getNumberOfBatches() == batches)
      continue;

    batches = solver->getNumberOfBatches();
    const double cost = solver->getPathCost();
    if (cost < cost_at_last_batch)
    {
      checked_prunings++;
      for (const NodePtr& node : solver->getStartTree()->getNodes())
      {
        const double f_hat = p.metrics->utopia(p.start, node) + p.metrics->utopia(node, p.goal);
        EXPECT_LE(f_hat, cost + 1e-9);
      }
    }
    cost_at_last_batch = cost;
  }
  ASSERT_TRUE(solver->solved());
  EXPECT_GT(checked_prunings, 0u);
}

TEST_F(BITStarTest, ReusesStartTree)
{
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);
  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, defaultParams(), solution, 0.5, 100000000));
  const double cost = solution->cost();

  // A new solver starting from the tree of the first one is solved before any search step
  BITStarPtr second = makeSolver(p);
  ASSERT_TRUE(second->config(defaultParams()));
  ASSERT_TRUE(second->addStartTree(solution->getTree()));
  ASSERT_TRUE(second->addGoal(p.goal));
  ASSERT_TRUE(second->solved());
  EXPECT_NEAR(second->getPathCost(), cost, 1e-9);

  // and it keeps improving (or at least does not get worse)
  PathPtr second_solution;
  EXPECT_TRUE(second->solve(second_solution, 100000000, 0.5));
  EXPECT_LE(second->getPathCost(), cost + 1e-9);
  expectValidPath(second_solution, p);
}

TEST_F(BITStarTest, RejectsStartAndGoalInCollision)
{
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);
  ASSERT_TRUE(solver->config(defaultParams()));

  Eigen::VectorXd in_wall(2);
  in_wall << 0.0, 1.0;
  NodePtr bad = std::make_shared<graph::core::Node>(in_wall, p.logger);
  EXPECT_FALSE(solver->addStart(bad));
  EXPECT_FALSE(solver->addGoal(bad));

  Eigen::VectorXd out_of_bounds(2);
  out_of_bounds << 2.0, 0.0;
  EXPECT_FALSE(solver->addGoal(std::make_shared<graph::core::Node>(out_of_bounds, p.logger)));
}

TEST_F(BITStarTest, UpdateBeforeProblemIsSetDoesNothing)
{
  Problem p = wallProblem();
  BITStarPtr solver = makeSolver(p);
  ASSERT_TRUE(solver->config(defaultParams()));
  PathPtr solution;
  EXPECT_FALSE(solver->update(solution));
  EXPECT_FALSE(solution);
  EXPECT_FALSE(solver->solve(solution, 10, 0.1));
}

TEST_F(BITStarTest, PluginCreatesSolver)
{
  Problem p = wallProblem();
  cnr_class_loader::MultiLibraryClassLoader loader(false);
  loader.loadLibrary(BIT_STAR_LIBRARY_NAME);
  std::shared_ptr<graph::core::TreeSolverPlugin> plugin =
      loader.createInstance<graph::core::TreeSolverPlugin>("graph::bit_star::BITStarPlugin");
  ASSERT_TRUE(plugin);

  graph::core::GoalCostFunctionPtr goal_cost_fcn = std::make_shared<graph::core::GoalCostFunctionBase>();
  ASSERT_TRUE(plugin->init(defaultParams(), p.metrics, p.checker, p.sampler, goal_cost_fcn, p.logger));
  graph::core::TreeSolverPtr solver = plugin->getSolver();
  ASSERT_TRUE(std::dynamic_pointer_cast<BITStar>(solver));

  PathPtr solution;
  ASSERT_TRUE(solver->computePath(p.start, p.goal, defaultParams(), solution, 0.5, 100000000));
  expectValidPath(solution, p);

  // Release the objects created by the library before unloading it
  solver.reset();
  plugin.reset();
}

}  // namespace
