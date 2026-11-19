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

// BIT* in the plane: a wall x in [-0.05, 0.05] with a slit |y| < 0.1, start (-1, 0.8), goal (1, 0.8).
// Prints the cost of the solution every time it improves, then the waypoints of the final path.
//
// Usage: bit_star_wall_example [max_time_s] [log_level]
//   max_time_s  planning time in seconds (default 2)
//   log_level   trace, debug, info, warn, error, fatal (default info)

#include <bit_star/solvers/bit_star.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/samplers/informed_sampler.h>

namespace
{
/// Wall |x| <= 0.05 with a slit |y| < 0.1.
class WallChecker : public graph::core::CollisionCheckerBase
{
public:
  explicit WallChecker(const graph::core::LoggerPtr& logger) : CollisionCheckerBase(logger, 0.005)
  {
  }
  bool check(const Eigen::VectorXd& q) override
  {
    return std::abs(q(0)) > 0.05 || std::abs(q(1)) < 0.1;
  }
  graph::core::CollisionCheckerPtr clone() override
  {
    return std::make_shared<WallChecker>(logger_);
  }
};
}  // namespace

int main(int argc, char** argv)
{
  const double max_time = (argc > 1) ? std::atof(argv[1]) : 2.0;

  graph::core::LogLevel level = graph::core::LogLevel::Info;
  if (argc > 2)
  {
    auto parsed = graph::core::logLevelFromString(argv[2]);
    if (not parsed)
    {
      std::cerr << "Invalid log level '" << argv[2] << "'. Use: trace, debug, info, warn, error, fatal" << std::endl;
      return 1;
    }
    level = *parsed;
  }
  graph::core::LoggerPtr logger = std::make_shared<graph::core::ConsoleLogger>("bit_star_wall_example", level);

  // Problem
  Eigen::VectorXd start(2), goal(2), lb(2), ub(2);
  start << -1.0, 0.8;
  goal << 1.0, 0.8;
  lb << -1.5, -1.5;
  ub << 1.5, 1.5;

  graph::core::CollisionCheckerPtr checker = std::make_shared<WallChecker>(logger);
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);
  // Foci on start and goal: BIT* shrinks the ellipsoid with SamplerBase::setCost() at each improvement
  graph::core::SamplerPtr sampler = std::make_shared<graph::core::InformedSampler>(start, goal, lb, ub, logger);

  // Parameters (graph_core does not read files: the application fills the ParamMap)
  graph::core::ParamMap params;
  params["samples_per_batch"] = int64_t(100);
  params["rewire_factor"] = 1.1;
  params["knearest"] = false;
  params["prune"] = true;
  params["prune_threshold"] = 0.05;
  params["utopia_tolerance"] = 0.001;

  graph::bit_star::BITStar solver(metrics, checker, sampler, logger);
  solver.config(params);
  graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start, logger);
  graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal, logger);
  if (not solver.addStart(start_node) || not solver.addGoal(goal_node))
    return 1;

  // Anytime loop: one update() is one step of the search (one edge, or a new batch)
  graph::core::PathPtr solution;
  const graph::core::graph_time_point tic = graph::core::graph_time::now();
  double elapsed = 0.0;
  while (elapsed < max_time && solver.canImprove())
  {
    if (solver.update(solution) && solution)
    {
      GC_INFO(logger, "t = " << elapsed << " s, cost " << solver.getCost() << ", batch " << solver.getNumberOfBatches()
                             << ", " << solver.getStartTree()->getNumberOfNodes() << " vertices, "
                             << solver.getNumberOfSamples() << " samples, " << solver.getNumberOfCheckedEdges()
                             << " edges checked");
    }
    elapsed = graph::core::toSeconds(graph::core::graph_time::now(), tic);
  }

  if (not solution)
  {
    GC_ERROR(logger, "No solution found in " << max_time << " s");
    return 1;
  }

  const double optimum = 2.0 * std::hypot(0.95, 0.7) + 0.1;
  GC_INFO(logger, "Final cost " << solution->cost() << " (optimum " << optimum << ", +"
                                << 100.0 * (solution->cost() / optimum - 1.0) << " %)");
  for (const Eigen::VectorXd& waypoint : solution->getWaypoints())
    std::cout << waypoint.transpose() << std::endl;
  return 0;
}
