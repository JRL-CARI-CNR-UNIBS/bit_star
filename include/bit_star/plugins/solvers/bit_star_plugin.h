#pragma once
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
#include <graph_core/plugins/solvers/tree_solver_plugin.h>

namespace graph
{
namespace bit_star
{
/**
 * @brief Plugin that exposes graph::bit_star::BITStar as a graph::core::TreeSolverPlugin, loadable by name with
 * cnr_class_loader (class name "graph::bit_star::BITStarPlugin", library "libbit_star.so").
 */
class BITStarPlugin : public graph::core::TreeSolverPlugin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BITStarPlugin() : TreeSolverPlugin()
  {
  }

  /**
   * @brief Creates the solver. The parameters are not read here: as for the other graph_core solvers, they are
   * passed later to TreeSolver::config() (or to TreeSolver::computePath()).
   */
  virtual bool init(const graph::core::ParamMap& params, const graph::core::MetricsPtr& metrics,
                    const graph::core::CollisionCheckerPtr& checker, const graph::core::SamplerPtr& sampler,
                    const graph::core::GoalCostFunctionPtr& goal_cost_fcn,
                    const graph::core::LoggerPtr& logger) override
  {
    (void)params;
    solver_ = std::make_shared<graph::bit_star::BITStar>(metrics, checker, sampler, goal_cost_fcn, logger);
    return true;
  }
};

}  // end namespace bit_star
}  // end namespace graph
