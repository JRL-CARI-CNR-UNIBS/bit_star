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

// Anytime comparison of BIT*, graph_core's RRT* and, if the package mirrt_star is found at build time, MI-RRT*
// (graph::mirrt_star::MultigoalSolver, here with a single goal). Problems (obstacles are axis-aligned boxes, any n >=
// 2):
//   wall2d:      wall |x| <= 0.05 with a slit |y| < 0.1 in [-1.5, 1.5]^2, from (-1, 0.8) to (1, 0.8);
//   cube<n>d:    obstacle [-1, 1]^n in [-2.5, 2.5]^n, from (-1.5, ..., -1.5) to (1.5, ..., 1.5);
//   twocubes<n>d: two boxes [-1, 1]^n stretched and shifted along x1, x1 in [0.05, 2.05] and [-2.05, -0.05], with
//                a passage |x1| < 0.05 between them, in [-2.5, 2.5]^n, from (-1.5, 0.6, 0, ...) to (1.5, 0.6, 0, ...).
//                The optimal path goes through the passage.
// All solvers get the same kind of InformedSampler (MI-RRT* uses only its bounds and builds its own informed
// samplers), Euclidean metrics and collision checker. For each solver, the program
// prints the median time to the first solution and, at each instant, the median (and worst) cost, the median number
// of iterations (calls to update(): one edge for BIT*, one sample for RRT* and MI-RRT*) and, for BIT*, the median
// number of batches.
//
// Usage: bit_star_compare_rrt_star [--runs N] [--max-distance D] [--solver LIST] [problem ...]
//   --runs          runs per solver and problem (default 7)
//   --max-distance  RRT* and MI-RRT* steering distance (default 1.0)
//   --solver        comma-separated list of bit, rrt, mirrt (default: all the available ones)
//   problem         wall2d, cube<n>d, twocubes<n>d (default: wall2d, cube3d/6d/12d/18d, twocubes2d/3d/6d/12d/18d)
// Instants: 0.1, 1, 5, 10, 20, 50 s, so every run lasts 50 s.

#include <bit_star/solvers/bit_star.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/samplers/informed_sampler.h>
#include <graph_core/solvers/rrt_star.h>
#ifdef BIT_STAR_WITH_MIRRT_STAR
#include <mirrt_star/solvers/mirrt_star.h>
#endif

namespace
{
using graph::core::CollisionCheckerPtr;
using graph::core::LoggerPtr;

constexpr double inf = std::numeric_limits<double>::infinity();

/// Axis-aligned boxes in any dimension: a configuration is in collision if it is strictly inside one of them.
class BoxesChecker : public graph::core::CollisionCheckerBase
{
public:
  using Box = std::pair<Eigen::VectorXd, Eigen::VectorXd>;  // lower and upper corner

  BoxesChecker(const LoggerPtr& logger, const std::vector<Box>& boxes)
    : CollisionCheckerBase(logger, 0.005), boxes_(boxes)
  {
  }
  bool check(const Eigen::VectorXd& q) override
  {
    for (const Box& box : boxes_)
      if ((q.array() > box.first.array()).all() && (q.array() < box.second.array()).all())
        return false;
    return true;
  }
  CollisionCheckerPtr clone() override
  {
    return std::make_shared<BoxesChecker>(logger_, boxes_);
  }

private:
  std::vector<Box> boxes_;
};

/// Planning problem and its optimal cost.
struct Problem
{
  std::string name;
  Eigen::VectorXd start, goal, lb, ub;
  CollisionCheckerPtr checker;
  double optimum;
};

/// Dimension n of a name "<prefix><n>d", 0 if the name does not match.
int dimension(const std::string& name, const std::string& prefix)
{
  if (name.size() < prefix.size() + 2 || name.compare(0, prefix.size(), prefix) != 0 || name.back() != 'd')
    return 0;
  const std::string digits = name.substr(prefix.size(), name.size() - prefix.size() - 1);
  if (digits.find_first_not_of("0123456789") != std::string::npos)
    return 0;
  const int n = std::atoi(digits.c_str());
  return (n >= 2) ? n : 0;
}

/// Returns false if the name is unknown.
bool makeProblem(const std::string& name, const LoggerPtr& logger, Problem& p)
{
  p.name = name;
  int n = 0;
  std::vector<BoxesChecker::Box> boxes;
  if (name == "wall2d")
  {
    p.start = Eigen::Vector2d(-1.0, 0.8);
    p.goal = Eigen::Vector2d(1.0, 0.8);
    p.lb = Eigen::VectorXd::Constant(2, -1.5);
    boxes.emplace_back(Eigen::Vector2d(-0.05, 0.1), Eigen::Vector2d(0.05, 10.0));
    boxes.emplace_back(Eigen::Vector2d(-0.05, -10.0), Eigen::Vector2d(0.05, -0.1));
    p.optimum = 2.0 * std::hypot(0.95, 0.7) + 0.1;  // through the corners of the slit
  }
  else if ((n = dimension(name, "cube")) > 0)
  {
    p.start = Eigen::VectorXd::Constant(n, -1.5);
    p.goal = Eigen::VectorXd::Constant(n, 1.5);
    p.lb = Eigen::VectorXd::Constant(n, -2.5);
    boxes.emplace_back(Eigen::VectorXd::Constant(n, -1.0), Eigen::VectorXd::Constant(n, 1.0));
    // Bending once on a (n-2)-face of the cube, e.g. at (-1, 1, 0, ..., 0)
    p.optimum = 2.0 * std::sqrt(0.5 * 0.5 + 2.5 * 2.5 + (n - 2) * 1.5 * 1.5);
  }
  else if ((n = dimension(name, "twocubes")) > 0)
  {
    const double half_gap = 0.05;
    p.start = Eigen::VectorXd::Zero(n);
    p.start(0) = -1.5;
    p.start(1) = 0.6;
    p.goal = p.start;
    p.goal(0) = 1.5;
    p.lb = Eigen::VectorXd::Constant(n, -2.5);
    Eigen::VectorXd lower = Eigen::VectorXd::Constant(n, -1.0);
    Eigen::VectorXd upper = Eigen::VectorXd::Constant(n, 1.0);
    lower(1) = half_gap;
    upper(1) = half_gap + 2.0;
    boxes.emplace_back(lower, upper);
    lower(1) = -half_gap - 2.0;
    upper(1) = -half_gap;
    boxes.emplace_back(lower, upper);
    // Through the passage, touching the edges x1 = half_gap of the upper box at x0 = -1 and x0 = 1
    p.optimum = 2.0 * std::hypot(0.5, 0.6 - half_gap) + 2.0;
  }
  else
  {
    return false;
  }
  p.ub = -p.lb;
  p.checker = std::make_shared<BoxesChecker>(logger, boxes);
  return true;
}

double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}
}  // namespace

int main(int argc, char** argv)
{
  int runs = 7;
  double max_distance = 1.0;
#ifdef BIT_STAR_WITH_MIRRT_STAR
  std::string solver_option = "bit,rrt,mirrt";
#else
  std::string solver_option = "bit,rrt";
#endif
  std::vector<std::string> names;
  for (int i = 1; i < argc; i++)
  {
    const std::string arg = argv[i];
    if (arg == "--runs" && i + 1 < argc)
      runs = std::max(1, std::atoi(argv[++i]));
    else if (arg == "--max-distance" && i + 1 < argc)
      max_distance = std::atof(argv[++i]);
    else if (arg == "--solver" && i + 1 < argc)
      solver_option = argv[++i];
    else if (arg.rfind("--", 0) == 0)
    {
      std::cerr << "usage: " << argv[0] << " [--runs N] [--max-distance D] [--solver bit,rrt,mirrt] [problem ...]\n";
      return 1;
    }
    else
      names.push_back(arg);
  }
  if (names.empty())
    names = { "wall2d",     "cube3d",     "cube6d",     "cube12d",     "cube18d",
              "twocubes2d", "twocubes3d", "twocubes6d", "twocubes12d", "twocubes18d" };
  std::vector<std::string> solver_names;
  for (size_t start = 0; start <= solver_option.size();)
  {
    size_t end = solver_option.find(',', start);
    if (end == std::string::npos)
      end = solver_option.size();
    const std::string token = solver_option.substr(start, end - start);
    start = end + 1;
    if (token == "bit")
      solver_names.push_back("BIT*");
    else if (token == "rrt")
      solver_names.push_back("RRT*");
#ifdef BIT_STAR_WITH_MIRRT_STAR
    else if (token == "mirrt")
      solver_names.push_back("MI-RRT*");
#endif
    else
    {
      std::cerr << "unknown or unavailable solver " << token << "\n";
      return 1;
    }
  }
  const std::vector<double> instants = { 0.1, 1.0, 5.0, 10.0, 20.0, 50.0 };  // [s]

  LoggerPtr logger = std::make_shared<graph::core::ConsoleLogger>("compare_rrt_star", graph::core::LogLevel::Error);

  graph::core::ParamMap params;
  params["max_distance"] = max_distance;
  params["utopia_tolerance"] = 0.0001;
  params["rewire_factor"] = 1.1;
  params["samples_per_batch"] = int64_t(100);

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "median over " << runs << " runs, RRT* max_distance " << max_distance
            << " (also MI-RRT*); at each instant: cost median [worst] (inf = no solution yet), iterations, BIT* "
               "batches\n";
  for (const std::string& name : names)
  {
    Problem problem;
    if (!makeProblem(name, logger, problem))
    {
      std::cerr << "unknown problem " << name << "\n";
      return 1;
    }
    for (const std::string& solver_name : solver_names)
    {
      const size_t m = instants.size();
      std::vector<std::vector<double>> costs(m), iterations(m), batches(m);
      std::vector<double> first_solution;
      for (int run = 0; run < runs; run++)
      {
        makeProblem(name, logger, problem);
        graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);
        graph::core::SamplerPtr sampler =
            std::make_shared<graph::core::InformedSampler>(problem.start, problem.goal, problem.lb, problem.ub, logger);
        std::shared_ptr<graph::bit_star::BITStar> bit_star;
        graph::core::TreeSolverPtr solver;
#ifdef BIT_STAR_WITH_MIRRT_STAR
        graph::mirrt_star::MultigoalSolverPtr mirrt_star;
#endif
        if (solver_name == "BIT*")
          solver = bit_star = std::make_shared<graph::bit_star::BITStar>(metrics, problem.checker, sampler, logger);
        else if (solver_name == "RRT*")
          solver = std::make_shared<graph::core::RRTStar>(metrics, problem.checker, sampler, logger);
#ifdef BIT_STAR_WITH_MIRRT_STAR
        else
          solver = mirrt_star = std::make_shared<graph::mirrt_star::MultigoalSolver>(
              metrics, problem.checker, sampler, std::make_shared<graph::core::GoalCostFunctionBase>(), logger);
#endif

        solver->config(params);
        solver->addStart(std::make_shared<graph::core::Node>(problem.start, logger));
        solver->addGoal(std::make_shared<graph::core::Node>(problem.goal, logger));
#ifdef BIT_STAR_WITH_MIRRT_STAR
        if (mirrt_star)
          mirrt_star->finalizeProblem();  // required by MI-RRT* after the last goal
#endif

        // Anytime loop, reading cost and counters at the given instants
        graph::core::PathPtr solution;
        const graph::core::graph_time_point tic = graph::core::graph_time::now();
        double first = inf;
        unsigned long iteration = 0;
        for (size_t k = 0; k < m;)
        {
          solver->update(solution);
          iteration++;
          const double t = graph::core::toSeconds(graph::core::graph_time::now(), tic);
          if (first == inf && solver->solved())
            first = t;
          for (; k < m && t >= instants[k]; k++)
          {
            costs[k].push_back(solver->solved() ? solver->getCost() : inf);
            iterations[k].push_back(static_cast<double>(iteration));
            batches[k].push_back(bit_star ? static_cast<double>(bit_star->getNumberOfBatches()) : 0.0);
          }
        }
        first_solution.push_back(first);
      }

      std::cout << name << "  " << solver_name << "  first solution " << 1000.0 * median(first_solution) << " ms\n";
      for (size_t k = 0; k < m; k++)
      {
        std::cout << "    " << std::setw(5) << std::setprecision(1) << instants[k] << " s: cost "
                  << std::setprecision(4) << median(costs[k]) << " ["
                  << *std::max_element(costs[k].begin(), costs[k].end()) << "]" << std::setprecision(0)
                  << ", iterations " << median(iterations[k]);
        if (solver_name == "BIT*")
          std::cout << ", batches " << median(batches[k]);
        std::cout << std::setprecision(4) << "\n";
      }
      std::cout << "    (optimum " << problem.optimum << ")" << std::endl;
    }
  }
  return 0;
}
