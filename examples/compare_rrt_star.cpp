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

// Anytime comparison of BIT* and graph_core's RRT* on two problems:
//   wall2d: wall |x| <= 0.05 with a slit |y| < 0.1 in [-1.5, 1.5]^2, from (-1, 0.8) to (1, 0.8);
//   cube3d: obstacle [-1, 1]^3 in [-2.5, 2.5]^3, from (-1.5, -1.5, -1.5) to (1.5, 1.5, 1.5).
// Both solvers get the same InformedSampler, Euclidean metrics and collision checker. For each solver, the program
// prints the median time to the first solution and the median (and worst) cost at some instants.
//
// Usage: bit_star_compare_rrt_star [runs] [max_distance]
//   runs          runs per solver and problem (default 7)
//   max_distance  RRT* steering distance (default 1.0)

#include <bit_star/solvers/bit_star.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/samplers/informed_sampler.h>
#include <graph_core/solvers/rrt_star.h>

namespace
{
using graph::core::CollisionCheckerPtr;
using graph::core::LoggerPtr;

constexpr double inf = std::numeric_limits<double>::infinity();

/// Wall |x| <= 0.05 with a slit |y| < 0.1.
class WallChecker : public graph::core::CollisionCheckerBase
{
public:
  explicit WallChecker(const LoggerPtr& logger) : CollisionCheckerBase(logger, 0.005)
  {
  }
  bool check(const Eigen::VectorXd& q) override
  {
    return std::abs(q(0)) > 0.05 || std::abs(q(1)) < 0.1;
  }
  CollisionCheckerPtr clone() override
  {
    return std::make_shared<WallChecker>(logger_);
  }
};

/// Planning problem and its optimal cost.
struct Problem
{
  std::string name;
  Eigen::VectorXd start, goal, lb, ub;
  CollisionCheckerPtr checker;
  double optimum;
};

Problem makeProblem(const std::string& name, const LoggerPtr& logger)
{
  Problem p;
  p.name = name;
  if (name == "wall2d")
  {
    p.start = Eigen::Vector2d(-1.0, 0.8);
    p.goal = Eigen::Vector2d(1.0, 0.8);
    p.lb = Eigen::VectorXd::Constant(2, -1.5);
    p.checker = std::make_shared<WallChecker>(logger);
    p.optimum = 2.0 * std::hypot(0.95, 0.7) + 0.1;  // through the corners of the slit
  }
  else
  {
    p.start = Eigen::VectorXd::Constant(3, -1.5);
    p.goal = Eigen::VectorXd::Constant(3, 1.5);
    p.lb = Eigen::VectorXd::Constant(3, -2.5);
    p.checker = std::make_shared<graph::core::Cube3dCollisionChecker>(logger, 1.0, 0.01);
    p.optimum = std::sqrt(35.0);  // bending once on an edge of the cube, e.g. at (-1, 1, 0)
  }
  p.ub = -p.lb;
  return p;
}

double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}
}  // namespace

int main(int argc, char** argv)
{
  const int runs = (argc > 1) ? std::max(1, std::atoi(argv[1])) : 7;
  const double max_distance = (argc > 2) ? std::atof(argv[2]) : 1.0;
  const std::vector<double> instants = { 0.05, 0.1, 0.25, 0.5, 1.0 };  // [s]

  LoggerPtr logger = std::make_shared<graph::core::ConsoleLogger>("compare_rrt_star", graph::core::LogLevel::Error);

  graph::core::ParamMap params;
  params["max_distance"] = max_distance;
  params["utopia_tolerance"] = 0.0001;
  params["rewire_factor"] = 1.1;
  params["samples_per_batch"] = int64_t(100);

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "median over " << runs << " runs; cost: median [worst], inf = no solution yet\n";
  for (const std::string name : { "wall2d", "cube3d" })
  {
    for (const std::string solver_name : { "BIT*", "RRT*" })
    {
      std::vector<std::vector<double>> costs(instants.size());
      std::vector<double> first_solution;
      Problem problem;
      for (int run = 0; run < runs; run++)
      {
        problem = makeProblem(name, logger);
        graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);
        graph::core::SamplerPtr sampler =
            std::make_shared<graph::core::InformedSampler>(problem.start, problem.goal, problem.lb, problem.ub, logger);
        graph::core::TreeSolverPtr solver;
        if (solver_name == "BIT*")
          solver = std::make_shared<graph::bit_star::BITStar>(metrics, problem.checker, sampler, logger);
        else
          solver = std::make_shared<graph::core::RRTStar>(metrics, problem.checker, sampler, logger);

        solver->config(params);
        solver->addStart(std::make_shared<graph::core::Node>(problem.start, logger));
        solver->addGoal(std::make_shared<graph::core::Node>(problem.goal, logger));

        // Anytime loop, reading the cost at the given instants
        graph::core::PathPtr solution;
        const graph::core::graph_time_point tic = graph::core::graph_time::now();
        double first = inf;
        for (size_t k = 0; k < instants.size();)
        {
          solver->update(solution);
          const double t = graph::core::toSeconds(graph::core::graph_time::now(), tic);
          if (first == inf && solver->solved())
            first = t;
          for (; k < instants.size() && t >= instants[k]; k++)
            costs[k].push_back(solver->solved() ? solver->getCost() : inf);
        }
        first_solution.push_back(first);
      }

      std::cout << std::setw(7) << name << "  " << solver_name << "  first solution " << 1000.0 * median(first_solution)
                << " ms";
      for (size_t k = 0; k < instants.size(); k++)
        std::cout << " | " << instants[k] << " s: " << median(costs[k]) << " ["
                  << *std::max_element(costs[k].begin(), costs[k].end()) << "]";
      std::cout << "  (optimum " << problem.optimum << ")\n";
    }
  }
  return 0;
}
