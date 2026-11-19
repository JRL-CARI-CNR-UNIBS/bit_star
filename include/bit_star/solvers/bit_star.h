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

#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <graph_core/datastructure/nearest_neighbors.h>
#include <graph_core/solvers/tree_solver.h>

namespace graph
{
namespace bit_star
{
class BITStar;
typedef std::shared_ptr<BITStar> BITStarPtr;

/**
 * @brief Batch Informed Trees (BIT*) solver for graph_core.
 *
 * BIT* (Gammell, Srinivasa, Barfoot, ICRA 2015 / IJRR 2020) is an anytime, asymptotically optimal planner. It
 * samples the configuration space in batches of collision-free states, which together with the tree vertices form
 * an implicit random geometric graph (RGG). The RGG is searched in order of the estimated solution cost through it,
 * as in A*: the edges are processed from a queue sorted by
 *
 *     g_T(v) + ĉ(v,x) + ĥ(x)
 *
 * where g_T is the cost-to-come in the tree, ĉ an admissible estimate of the edge cost (MetricsBase::utopia()) and
 * ĥ an admissible estimate of the cost-to-go (utopia towards the goal). The true edge cost and the collision check
 * are computed only when an edge is taken from the queue (lazy evaluation). When the queue holds no edge that can
 * improve the current solution the batch ends: the samples and vertices that cannot improve the solution are
 * pruned, a new batch of samples is drawn from the informed set, and the search restarts from the current tree.
 *
 * Integration with graph_core:
 * - the tree is a regular graph::core::Tree (start_tree_), so the solution is a regular graph::core::Path;
 * - the unconnected samples live in a separate graph::core::KdTree, and the tree vertices are indexed in a second
 *   one (Tree::nearK() uses its own number of neighbours);
 * - the per-vertex data needed by the search (cached g_T, ĝ, ĥ, queue bookkeeping) are kept in a side table
 *   indexed by node, not in the graph_core nodes.
 *
 * One call to update() performs one step of the search: it expands the vertices whose queue value is not worse
 * than the best edge, then processes the best edge. When both queues are empty, update() draws a new batch
 * instead. solve() and computePath() therefore need a large max_iter (one iteration ≈ one edge).
 *
 * The sampler given to the solver defines the informed set. Use a graph::core::InformedSampler whose foci are the
 * start and goal configurations: BIT* calls SamplerBase::setCost() with the path cost at each improvement.
 * Any other sampler works too, since the samples that cannot improve the solution are rejected anyway (but then
 * most of the samples are wasted once a solution is found).
 *
 * Parameters (read by config(), in addition to the ones of graph::core::TreeSolver):
 * | Name | Type | Default | Description |
 * |---|---|---|---|
 * | `samples_per_batch` | int | 100 | number of collision-free samples drawn at each batch |
 * | `sample_attempts_factor` | int | 10 | at most `sample_attempts_factor * samples_per_batch` draws per batch |
 * | `rewire_factor` | double | 1.1 | η, multiplies the RGG radius (or the number of neighbours) |
 * | `knearest` | bool | false | use the k-nearest RGG instead of the r-disc RGG |
 * | `prune` | bool | true | prune samples and vertices that cannot improve the solution |
 * | `prune_threshold` | double | 0.05 | prune only if the cost decreased by this fraction since the last pruning |
 * | `direct_connection` | bool | true | try the edge start-goal when the problem is set, as TreeSolver does |
 *
 * Without `direct_connection`, the goal is reached only through the RGG: the edge start-goal exists only if the
 * goal is within the radius (or among the k nearest neighbours) of the start, so even in free space the first
 * solution is a polyline that converges to the straight line as the batches go on (standard BIT* behaviour).
 *
 * `max_distance` and `extend` of TreeSolver are not used: BIT* connects samples with straight edges and never
 * steers.
 */
class BITStar : public graph::core::TreeSolver
{
protected:
  /**
   * @brief Search data attached to a node of the tree or to a sample.
   */
  struct Vertex
  {
    graph::core::NodePtr node;  ///< graph_core node (owned also by the tree or by the samples KdTree)
    uint64_t id;                ///< unique and never reused identifier, used by the invalid-edge cache
    double g_hat;               ///< ĝ: admissible estimate of the cost from the start
    double h_hat;               ///< ĥ: admissible estimate of the cost to the goal
    double g = std::numeric_limits<double>::infinity();  ///< g_T: cost-to-come in the tree, infinity for samples
    bool in_tree = false;          ///< true if the node is a tree vertex, false if it is an unconnected sample
    bool is_new = false;           ///< true if added to the tree during the current batch (not in V_old)
    bool in_vertex_queue = false;  ///< true if waiting in the vertex queue (not yet expanded in this batch)
    uint32_t version = 0;          ///< incremented when g changes; queue entries with an older version are stale
    std::unordered_set<Vertex*> queued_targets;    ///< targets of the edges from this vertex still in the edge queue
    std::unordered_set<uint64_t> invalid_targets;  ///< ids of the vertices/samples reachable only by colliding edges

    /// Lower bound of the cost of a solution through this vertex (f̂ = ĝ + ĥ).
    double fHat() const
    {
      return g_hat + h_hat;
    }
  };

  /**
   * @brief Entry of the vertex queue, sorted by g_T(v) + ĥ(v).
   */
  struct VertexEntry
  {
    double key;        ///< g_T(v) + ĥ(v) when the entry was pushed
    double g;          ///< g_T(v), tie-breaking
    Vertex* vertex;    ///< vertex to expand
    uint32_t version;  ///< vertex version when the entry was pushed
  };

  /**
   * @brief Entry of the edge queue, sorted lexicographically by (g_T(v) + ĉ + ĥ(x), g_T(v) + ĉ, g_T(v)).
   */
  struct EdgeEntry
  {
    double key;        ///< g_T(v) + ĉ(v,x) + ĥ(x) when the entry was pushed
    double g_plus_c;   ///< g_T(v) + ĉ(v,x), first tie-breaking
    double g;          ///< g_T(v), second tie-breaking
    double c_hat;      ///< ĉ(v,x)
    Vertex* parent;    ///< source vertex (in the tree)
    Vertex* child;     ///< target vertex or sample
    uint32_t version;  ///< parent version when the entry was pushed
  };

  /// Min-heap order on VertexEntry.
  struct VertexEntryGreater
  {
    bool operator()(const VertexEntry& a, const VertexEntry& b) const
    {
      if (a.key != b.key)
        return a.key > b.key;
      return a.g > b.g;
    }
  };

  /// Min-heap order on EdgeEntry.
  struct EdgeEntryGreater
  {
    bool operator()(const EdgeEntry& a, const EdgeEntry& b) const
    {
      if (a.key != b.key)
        return a.key > b.key;
      if (a.g_plus_c != b.g_plus_c)
        return a.g_plus_c > b.g_plus_c;
      return a.g > b.g;
    }
  };

  // Parameters
  unsigned int samples_per_batch_ = 100;      ///< collision-free samples drawn at each batch
  unsigned int sample_attempts_factor_ = 10;  ///< maximum draws per batch, relative to samples_per_batch_
  double rewire_factor_ = 1.1;                ///< η, scales the RGG radius or the number of neighbours
  bool knearest_ = false;                     ///< use the k-nearest RGG instead of the r-disc RGG
  bool prune_ = true;                         ///< prune samples and vertices that cannot improve the solution
  double prune_threshold_ = 0.05;             ///< minimum relative cost decrease between two prunings
  bool direct_connection_ = true;             ///< try the edge start-goal when the problem is set

  // Search state
  std::unordered_map<const graph::core::Node*, std::unique_ptr<Vertex>> vertices_;  ///< tree vertices and samples
  graph::core::NearestNeighborsPtr samples_;  ///< unconnected samples (the goal is a sample until connected)
  graph::core::NearestNeighborsPtr tree_nn_;  ///< tree vertices; Tree::nearK() has its own k, so BIT* keeps its index
  Vertex* start_vertex_ = nullptr;            ///< vertex of the tree root
  Vertex* goal_vertex_ = nullptr;             ///< vertex (or sample) of the goal
  std::priority_queue<VertexEntry, std::vector<VertexEntry>, VertexEntryGreater> vertex_queue_;  ///< Q_V
  std::priority_queue<EdgeEntry, std::vector<EdgeEntry>, EdgeEntryGreater> edge_queue_;          ///< Q_E
  double radius_ = std::numeric_limits<double>::infinity();  ///< current r-disc radius
  unsigned int k_neighbours_ = 0;                            ///< current number of neighbours (k-nearest RGG)
  double cost_at_last_prune_ = std::numeric_limits<double>::infinity();  ///< path cost at the last pruning
  uint64_t next_id_ = 0;                                                 ///< next Vertex::id

  // Statistics
  unsigned int batches_ = 0;           ///< number of batches drawn
  unsigned long edges_processed_ = 0;  ///< edges taken from the edge queue
  unsigned long edges_checked_ = 0;    ///< edges collision-checked
  unsigned long rewirings_ = 0;        ///< tree vertices that got a better parent

  /**
   * @brief Initialises the search state (start vertex and goal sample) and, if direct_connection_ is set, tries
   * the edge start-goal (TreeSolver::setProblem() does the same through Tree::connectToNode(), which would add
   * intermediate nodes unknown to BIT*).
   */
  virtual bool setProblem(const double& max_time = std::numeric_limits<double>::infinity()) override;

  virtual void printMyself(std::ostream& os) const override;

  /** @brief Removes all the search data (vertices, samples, queues and statistics). */
  void clearSearch();

  /**
   * @brief Rebuilds the search data from start_tree_: every tree node becomes a vertex with g = cost-to-come,
   * and the goal becomes a sample if it is not in the tree.
   */
  bool initSearchFromTree();

  /** @brief Creates the search data of a node. The node must not have one already. */
  Vertex* createVertex(const graph::core::NodePtr& node);

  /** @brief Returns the search data of a node, or nullptr if the node is unknown to the solver. */
  Vertex* findVertex(const graph::core::Node* node) const;

  /** @brief Path cost of the current solution (g_T of the goal), infinity if not solved. */
  double bestCost() const;

  /**
   * @brief Ends the current batch: prunes (if enabled and worth it), draws a new batch of samples, puts all the
   * tree vertices in the vertex queue and updates the RGG radius.
   */
  void newBatch();

  /**
   * @brief Draws samples_per_batch_ collision-free samples that can improve the current solution.
   * @return the number of samples added
   */
  unsigned int sampleBatch();

  /**
   * @brief Removes the samples with f̂ >= cost and the vertices with f̂ > cost. The descendants of a pruned vertex
   * that can still improve the solution are recycled as samples.
   * @return the number of nodes removed from the tree
   */
  unsigned int prune(const double& cost);

  /** @brief Updates radius_ and k_neighbours_ from the number of states and the measure of the informed set. */
  void updateRadius();

  /** @brief λ(X_f̂)/ζ_n: measure of the informed set divided by the measure of the unit n-ball. */
  double informedSpecificVolume() const;

  /** @brief Neighbours of a configuration among the samples (in_tree = false) or the tree vertices (true). */
  std::vector<Vertex*> neighbours(const Vertex* v, const bool in_tree) const;

  /** @brief Queues the outgoing edges of a vertex (Expand Vertex in the paper). */
  void expandVertex(Vertex* v);

  /** @brief Pushes the edge (parent, child) in the edge queue and records it in parent->queued_targets. */
  void queueEdge(Vertex* parent, Vertex* child, const double& c_hat);

  /** @brief Pushes the vertex in the vertex queue. */
  void queueVertex(Vertex* v);

  /** @brief Empties both queues and the per-vertex queue bookkeeping. */
  void clearQueues();

  /** @brief Pops the stale entries from the top of the vertex queue. @return the best valid entry, or nullptr. */
  const VertexEntry* topVertex();

  /** @brief Pops the stale entries from the top of the edge queue. @return the best valid entry, or nullptr. */
  const EdgeEntry* topEdge();

  /**
   * @brief Evaluates an edge taken from the queue: true cost, collision check, and connection to the tree if it
   * improves the cost-to-come of the target.
   * @return false if the batch is over (the edge cannot improve the solution), true otherwise
   */
  bool processEdge(const EdgeEntry& edge);

  /**
   * @brief Connects child to parent with an edge of cost c, replacing the previous parent of child if it was
   * already in the tree, and propagates the new cost-to-come to the descendants.
   */
  void connect(Vertex* parent, Vertex* child, const double& c);

  /** @brief Recomputes g of the descendants of v and re-queues their vertex and edge entries. */
  void propagateCost(Vertex* v);

  /** @brief True if ancestor is v or one of its ancestors in the tree. */
  bool isAncestor(const Vertex* ancestor, const Vertex* v) const;

  /**
   * @brief Updates solution_, the costs and the sampler if the goal cost-to-come improved.
   * @return true if the solution was found or improved
   */
  bool updateSolution();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BITStar(const graph::core::MetricsPtr& metrics, const graph::core::CollisionCheckerPtr& checker,
          const graph::core::SamplerPtr& sampler, const graph::core::GoalCostFunctionPtr& goal_cost_fcn,
          const graph::core::LoggerPtr& logger)
    : TreeSolver(metrics, checker, sampler, goal_cost_fcn, logger)
  {
  }

  BITStar(const graph::core::MetricsPtr& metrics, const graph::core::CollisionCheckerPtr& checker,
          const graph::core::SamplerPtr& sampler, const graph::core::LoggerPtr& logger)
    : TreeSolver(metrics, checker, sampler, logger)
  {
  }

  /** @brief Reads the parameters (see the class description) and resets the solver status. */
  virtual bool config(const graph::core::ParamMap& params) override;

  /**
   * @brief One step of the search (see the class description).
   * @param solution set to the current solution (nullptr if not solved yet)
   * @return true if the solution was found or improved in this step, or if it is already optimal within
   * utopia_tolerance
   */
  virtual bool update(graph::core::PathPtr& solution) override;

  /**
   * @brief Adds a configuration to the samples (if it is collision-free and can improve the solution), then
   * performs one step of the search.
   */
  virtual bool update(const Eigen::VectorXd& configuration, graph::core::PathPtr& solution) override;

  /** @brief Same as update(configuration, solution) using the node configuration. */
  virtual bool update(const graph::core::NodePtr& n, graph::core::PathPtr& solution) override;

  /**
   * @brief Runs update() until the solution is optimal within utopia_tolerance, max_iter steps are done, or
   * max_time is elapsed. Unlike TreeSolver::solve(), it keeps improving the solution after the first one.
   * @return true if a solution exists
   */
  virtual bool solve(graph::core::PathPtr& solution, const unsigned int& max_iter = 100,
                     const double& max_time = std::numeric_limits<double>::infinity()) override;

  /** @brief Creates the tree rooted at start_node. The start must be in bounds and collision-free. */
  virtual bool addStart(const graph::core::NodePtr& start_node,
                        const double& max_time = std::numeric_limits<double>::infinity()) override;

  /** @brief Sets the goal. The goal must be in bounds and collision-free. */
  virtual bool addGoal(const graph::core::NodePtr& goal_node,
                       const double& max_time = std::numeric_limits<double>::infinity()) override;

  /**
   * @brief Starts from an existing tree (e.g. from a previous query). The connections of the tree are assumed
   * collision-free and their cost up to date.
   */
  virtual bool addStartTree(const graph::core::TreePtr& start_tree,
                            const double& max_time = std::numeric_limits<double>::infinity()) override;

  /** @brief Removes start, goal, tree and search data. */
  virtual void resetProblem() override;

  /** @brief Sets the solution and its tree, and rebuilds the search data from that tree. */
  virtual bool setSolution(const graph::core::PathPtr& solution) override;

  /** @brief Imports the problem from another solver and rebuilds the search data from its tree. */
  virtual bool importFromSolver(const graph::core::TreeSolverPtr& solver) override;

  /** @brief Cost-to-come g_T cached by the solver, infinity if the node is not a tree vertex. */
  double getCostToCome(const graph::core::NodePtr& node) const;

  /** @brief Number of unconnected samples. */
  unsigned int getNumberOfSamples() const
  {
    return samples_ ? samples_->size() : 0;
  }

  /** @brief Number of batches drawn since the problem was set. */
  unsigned int getNumberOfBatches() const
  {
    return batches_;
  }

  /** @brief Number of edges taken from the edge queue. */
  unsigned long getNumberOfProcessedEdges() const
  {
    return edges_processed_;
  }

  /** @brief Number of edges collision-checked. */
  unsigned long getNumberOfCheckedEdges() const
  {
    return edges_checked_;
  }

  /** @brief Number of times a tree vertex got a better parent. */
  unsigned long getNumberOfRewirings() const
  {
    return rewirings_;
  }

  /** @brief Current radius of the r-disc RGG (infinity before the first batch). */
  double getRadius() const
  {
    return radius_;
  }

  /** @brief Current number of neighbours of the k-nearest RGG. */
  unsigned int getKNeighbours() const
  {
    return k_neighbours_;
  }
};

}  // end namespace bit_star
}  // end namespace graph
