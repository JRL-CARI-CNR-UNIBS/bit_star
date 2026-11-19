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

#include <graph_core/datastructure/kdtree.h>
#include <graph_core/datastructure/vector.h>

namespace graph
{
namespace bit_star
{
using graph::core::ConnectionPtr;
using graph::core::NodePtr;

namespace
{
constexpr double inf = std::numeric_limits<double>::infinity();

/// Parent of a tree node, nullptr for the root or an unconnected node.
const graph::core::Node* parentOf(const graph::core::Node* node)
{
  if (node->getParentConnectionsSize() == 0)
    return nullptr;
  return node->parentConnection(0)->getParent().get();
}
}  // namespace

// ---------------------------------------------------------------------------------------------------------------
// Configuration and problem definition
// ---------------------------------------------------------------------------------------------------------------

bool BITStar::config(const graph::core::ParamMap& params)
{
  if (not TreeSolver::config(params))
    return false;

  int samples_per_batch;
  graph::core::get_param(logger_, params_, "samples_per_batch", samples_per_batch, 100);
  if (samples_per_batch < 1)
  {
    GC_WARN(logger_, "samples_per_batch should be >= 1, set equal to 1");
    samples_per_batch = 1;
  }
  samples_per_batch_ = samples_per_batch;

  int sample_attempts_factor;
  graph::core::get_param(logger_, params_, "sample_attempts_factor", sample_attempts_factor, 10);
  if (sample_attempts_factor < 1)
  {
    GC_WARN(logger_, "sample_attempts_factor should be >= 1, set equal to 1");
    sample_attempts_factor = 1;
  }
  sample_attempts_factor_ = sample_attempts_factor;

  graph::core::get_param(logger_, params_, "rewire_factor", rewire_factor_, 1.1);
  if (rewire_factor_ <= 0.0)
  {
    GC_WARN(logger_, "rewire_factor should be > 0, set equal to 1.1");
    rewire_factor_ = 1.1;
  }
  else if (rewire_factor_ < 1.0)
    GC_WARN(logger_, "rewire_factor < 1: BIT* is no longer guaranteed to be asymptotically optimal");

  graph::core::get_param(logger_, params_, "knearest", knearest_, false);
  graph::core::get_param(logger_, params_, "prune", prune_, true);
  graph::core::get_param(logger_, params_, "direct_connection", direct_connection_, true);

  graph::core::get_param(logger_, params_, "prune_threshold", prune_threshold_, 0.05);
  if (prune_threshold_ < 0.0 || prune_threshold_ >= 1.0)
  {
    GC_WARN(logger_, "prune_threshold should be in [0, 1), set equal to 0.05");
    prune_threshold_ = 0.05;
  }

  solved_ = false;
  return true;
}

bool BITStar::addStart(const NodePtr& start_node, const double& max_time)
{
  if (not configured_)
  {
    GC_ERROR(logger_, "Solver is not configured!");
    return false;
  }
  if (not sampler_->inBounds(start_node->getConfiguration()))
  {
    GC_WARN(logger_, "Start not in bounds");
    return false;
  }
  if (not checker_->check(start_node->getConfiguration()))
  {
    GC_WARN(logger_, "Start in collision");
    return false;
  }

  solved_ = false;
  start_tree_ =
      std::make_shared<graph::core::Tree>(start_node, max_distance_, checker_, metrics_, logger_, use_kdtree_);
  setProblem(max_time);
  return true;
}

bool BITStar::addGoal(const NodePtr& goal_node, const double& max_time)
{
  if (not configured_)
  {
    GC_ERROR(logger_, "Solver is not configured!");
    return false;
  }
  if (not sampler_->inBounds(goal_node->getConfiguration()))
  {
    GC_WARN(logger_, "Goal not in bounds");
    return false;
  }
  if (not checker_->check(goal_node->getConfiguration()))
  {
    GC_WARN(logger_, "Goal in collision");
    return false;
  }

  solved_ = false;
  goal_node_ = goal_node;
  goal_cost_ = goal_cost_fcn_->cost(goal_node);
  setProblem(max_time);
  return true;
}

bool BITStar::addStartTree(const graph::core::TreePtr& start_tree, const double& max_time)
{
  if (not start_tree)
  {
    GC_ERROR(logger_, "Start tree is empty");
    return false;
  }
  start_tree_ = start_tree;
  solved_ = false;
  setProblem(max_time);
  return true;
}

bool BITStar::setProblem(const double& max_time)
{
  (void)max_time;  // a single edge is checked at most
  problem_set_ = false;
  if (not start_tree_ || not goal_node_)
    return false;

  can_improve_ = true;
  solved_ = false;
  solution_ = nullptr;
  path_cost_ = inf;
  goal_cost_ = goal_cost_fcn_->cost(goal_node_);
  cost_ = inf;
  best_utopia_ =
      goal_cost_ + metrics_->utopia(start_tree_->getRoot()->getConfiguration(), goal_node_->getConfiguration());

  if (not initSearchFromTree())
    return false;

  problem_set_ = true;

  if (direct_connection_ && not goal_vertex_->in_tree)
  {
    const Eigen::VectorXd& start = start_vertex_->node->getConfiguration();
    const Eigen::VectorXd& goal = goal_node_->getConfiguration();
    const double c = metrics_->cost(start, goal);
    if (std::isfinite(c))
    {
      edges_checked_++;
      if (checker_->checkConnection(start, goal))
      {
        connect(start_vertex_, goal_vertex_, c);
        clearQueues();  // the search starts with the first batch, which queues all the vertices
      }
      else
      {
        start_vertex_->invalid_targets.insert(goal_vertex_->id);
        goal_vertex_->invalid_targets.insert(start_vertex_->id);
      }
    }
  }

  updateSolution();  // the goal may already be in the tree
  return true;
}

void BITStar::resetProblem()
{
  clearSearch();
  goal_node_.reset();
  start_tree_.reset();
  solution_.reset();
  problem_set_ = false;
  solved_ = false;
  can_improve_ = true;
  path_cost_ = inf;
  cost_ = inf;
}

bool BITStar::setSolution(const graph::core::PathPtr& solution)
{
  if (not TreeSolver::setSolution(solution))
    return false;

  // TreeSolver::setSolution() has set start_tree_, goal_node_ and the costs
  if (not initSearchFromTree())
  {
    problem_set_ = false;
    return false;
  }
  return true;
}

bool BITStar::importFromSolver(const graph::core::TreeSolverPtr& solver)
{
  if (this == solver.get())
    return true;

  if (not TreeSolver::importFromSolver(solver))
    return false;

  if (start_tree_ && goal_node_)
  {
    if (not initSearchFromTree())
    {
      problem_set_ = false;
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------------------------------------------
// Search data
// ---------------------------------------------------------------------------------------------------------------

void BITStar::clearSearch()
{
  clearQueues();
  vertices_.clear();
  samples_.reset();
  tree_nn_.reset();
  start_vertex_ = nullptr;
  goal_vertex_ = nullptr;
  radius_ = inf;
  k_neighbours_ = 0;
  cost_at_last_prune_ = inf;
  batches_ = 0;
  edges_processed_ = 0;
  edges_checked_ = 0;
  rewirings_ = 0;
}

bool BITStar::initSearchFromTree()
{
  clearSearch();

  if (use_kdtree_)
  {
    samples_ = std::make_shared<graph::core::KdTree>(logger_);
    tree_nn_ = std::make_shared<graph::core::KdTree>(logger_);
  }
  else
  {
    samples_ = std::make_shared<graph::core::Vector>(logger_);
    tree_nn_ = std::make_shared<graph::core::Vector>(logger_);
  }

  const NodePtr& root = start_tree_->getRoot();
  for (const NodePtr& node : start_tree_->getNodes())
  {
    Vertex* v = createVertex(node);
    v->in_tree = true;
    v->g = (node == root) ? 0.0 : start_tree_->costToNode(node);
    if (not std::isfinite(v->g))
    {
      GC_ERROR(logger_, "A node of the start tree has no valid path from the root");
      return false;
    }
    tree_nn_->insert(node);
  }

  start_vertex_ = findVertex(root.get());
  if (not start_vertex_)
  {
    GC_ERROR(logger_, "The root is not among the nodes of the start tree");
    return false;
  }

  goal_vertex_ = findVertex(goal_node_.get());
  if (not goal_vertex_)
  {
    goal_vertex_ = createVertex(goal_node_);
    samples_->insert(goal_node_);
  }
  return true;
}

BITStar::Vertex* BITStar::createVertex(const NodePtr& node)
{
  auto vertex = std::make_unique<Vertex>();
  vertex->node = node;
  vertex->id = next_id_++;
  vertex->g_hat = metrics_->utopia(start_tree_->getRoot()->getConfiguration(), node->getConfiguration());
  vertex->h_hat = metrics_->utopia(node->getConfiguration(), goal_node_->getConfiguration());

  Vertex* v = vertex.get();
  vertices_[node.get()] = std::move(vertex);
  return v;
}

BITStar::Vertex* BITStar::findVertex(const graph::core::Node* node) const
{
  auto it = vertices_.find(node);
  return (it == vertices_.end()) ? nullptr : it->second.get();
}

double BITStar::bestCost() const
{
  return (goal_vertex_ && goal_vertex_->in_tree) ? goal_vertex_->g : inf;
}

double BITStar::getCostToCome(const NodePtr& node) const
{
  const Vertex* v = findVertex(node.get());
  return (v && v->in_tree) ? v->g : inf;
}

// ---------------------------------------------------------------------------------------------------------------
// Batches, sampling and pruning
// ---------------------------------------------------------------------------------------------------------------

void BITStar::newBatch()
{
  clearQueues();

  const double c_best = bestCost();
  unsigned int pruned = 0;
  if (prune_ && c_best < inf && c_best < (1.0 - prune_threshold_) * cost_at_last_prune_)
  {
    pruned = prune(c_best);
    cost_at_last_prune_ = c_best;
  }
  else
  {
    // The samples connected during the batch are only marked as deleted in the KdTree (its rebuild threshold is
    // infinite by default): rebuild it, otherwise the queries keep visiting them. prune() rebuilds it anyway.
    std::vector<NodePtr> sample_nodes = samples_->getNodes();
    samples_->clear();
    for (const NodePtr& node : sample_nodes)
      samples_->insert(node);
  }

  const unsigned int sampled = sampleBatch();
  batches_++;
  updateRadius();

  for (auto& item : vertices_)
  {
    Vertex* v = item.second.get();
    if (not v->in_tree)
      continue;
    v->is_new = false;  // V_old = V
    queueVertex(v);
  }

  GC_DEBUG(logger_, "BIT* batch " << batches_ << ": " << sampled << " new samples, " << pruned << " vertices pruned, "
                                  << tree_nn_->size() << " vertices, " << samples_->size() << " samples, radius "
                                  << radius_ << ", k " << k_neighbours_ << ", cost " << c_best);
}

unsigned int BITStar::sampleBatch()
{
  const double c_best = bestCost();
  const Eigen::VectorXd& start = start_tree_->getRoot()->getConfiguration();
  const Eigen::VectorXd& goal = goal_node_->getConfiguration();

  const unsigned int max_attempts = sample_attempts_factor_ * samples_per_batch_;
  unsigned int added = 0;
  for (unsigned int attempt = 0; attempt < max_attempts && added < samples_per_batch_; attempt++)
  {
    Eigen::VectorXd q = sampler_->sample();

    // Reject the samples that cannot improve the solution before the (more expensive) collision check
    if (metrics_->utopia(start, q) + metrics_->utopia(q, goal) >= c_best)
      continue;
    if (not checker_->check(q))
      continue;

    NodePtr node = std::make_shared<graph::core::Node>(q, logger_);
    createVertex(node);
    samples_->insert(node);
    added++;
  }
  return added;
}

unsigned int BITStar::prune(const double& cost)
{
  // 1. Samples: keep only the ones that can improve the solution (and always the goal, while unconnected)
  std::vector<NodePtr> sample_nodes = samples_->getNodes();
  samples_->clear();
  for (const NodePtr& node : sample_nodes)
  {
    Vertex* v = findVertex(node.get());
    if (v == goal_vertex_ || v->fHat() < cost)
      samples_->insert(node);
    else
      vertices_.erase(node.get());
  }

  // 2. Tree: never prune the current solution, whatever the numerical noise on f̂
  std::unordered_set<const Vertex*> solution_path;
  if (goal_vertex_->in_tree)
  {
    for (const graph::core::Node* n = goal_node_.get(); n; n = parentOf(n))
      solution_path.insert(findVertex(n));
  }

  // Collect the subtrees rooted at the vertices with f̂ > cost
  std::vector<Vertex*> removed;
  std::vector<Vertex*> stack{ start_vertex_ };
  while (not stack.empty())
  {
    Vertex* u = stack.back();
    stack.pop_back();
    for (const ConnectionPtr& conn : u->node->getChildConnections())
    {
      Vertex* child = findVertex(conn->getChild().get());
      if (child->fHat() > cost && not solution_path.count(child))
      {
        // The whole subtree loses its path from the root
        std::vector<Vertex*> subtree{ child };
        while (not subtree.empty())
        {
          Vertex* s = subtree.back();
          subtree.pop_back();
          removed.push_back(s);
          for (const ConnectionPtr& c : s->node->getChildConnections())
            subtree.push_back(findVertex(c->getChild().get()));
        }
      }
      else
        stack.push_back(child);
    }
  }

  // Detach the removed vertices; recycle as samples the ones that can still improve the solution
  for (Vertex* v : removed)
    start_tree_->removeNode(v->node);  // disconnects the node and removes it from the tree

  for (Vertex* v : removed)
  {
    if (v->fHat() < cost)
    {
      v->in_tree = false;
      v->is_new = false;
      v->g = inf;
      v->version++;
      samples_->insert(v->node);
    }
    else
      vertices_.erase(v->node.get());
  }

  // 3. Rebuild the nearest-neighbour index of the tree vertices
  tree_nn_->clear();
  for (auto& item : vertices_)
  {
    if (item.second->in_tree)
      tree_nn_->insert(item.second->node);
  }

  return removed.size();
}

double BITStar::informedSpecificVolume() const
{
  // Measure of the bounding box divided by the measure of the unit n-ball, as in SamplerBase
  const Eigen::VectorXd& lb = sampler_->getLB();
  const Eigen::VectorXd& ub = sampler_->getUB();
  const double n = static_cast<double>(dof_);
  double box = std::tgamma(0.5 * n + 1.0) / std::pow(M_PI, 0.5 * n);
  for (unsigned int idx = 0; idx < dof_; idx++)
    box *= (ub(idx) - lb(idx));

  // InformedSampler gives the measure of the ellipsoid (product of the semi-axes) once the cost is finite
  const double sampler_volume = sampler_->getSpecificVolume();
  if (sampler_volume > 0.0 && std::isfinite(sampler_volume))
    return std::min(box, sampler_volume);
  return box;
}

void BITStar::updateRadius()
{
  const double n = static_cast<double>(dof_);
  const double q = std::max(2.0, static_cast<double>(tree_nn_->size() + samples_->size()));

  // Gammell et al., IJRR 2020, eq. (5) and (6)
  radius_ = rewire_factor_ * 2.0 * std::pow(1.0 + 1.0 / n, 1.0 / n) * std::pow(informedSpecificVolume(), 1.0 / n) *
            std::pow(std::log(q) / q, 1.0 / n);
  k_neighbours_ = static_cast<unsigned int>(std::ceil(rewire_factor_ * M_E * (1.0 + 1.0 / n) * std::log(q)));
}

// ---------------------------------------------------------------------------------------------------------------
// Queues
// ---------------------------------------------------------------------------------------------------------------

void BITStar::queueVertex(Vertex* v)
{
  v->in_vertex_queue = true;
  vertex_queue_.push(VertexEntry{ v->g + v->h_hat, v->g, v, v->version });
}

void BITStar::queueEdge(Vertex* parent, Vertex* child, const double& c_hat)
{
  parent->queued_targets.insert(child);
  edge_queue_.push(EdgeEntry{ parent->g + c_hat + child->h_hat, parent->g + c_hat, parent->g, c_hat, parent, child,
                              parent->version });
}

void BITStar::clearQueues()
{
  vertex_queue_ = decltype(vertex_queue_)();
  edge_queue_ = decltype(edge_queue_)();
  for (auto& item : vertices_)
  {
    item.second->in_vertex_queue = false;
    item.second->queued_targets.clear();
  }
}

const BITStar::VertexEntry* BITStar::topVertex()
{
  while (not vertex_queue_.empty())
  {
    const VertexEntry& top = vertex_queue_.top();
    if (top.vertex->in_vertex_queue && top.vertex->in_tree && top.version == top.vertex->version)
      return &top;
    vertex_queue_.pop();  // stale: already expanded, or g changed and a newer entry exists
  }
  return nullptr;
}

const BITStar::EdgeEntry* BITStar::topEdge()
{
  while (not edge_queue_.empty())
  {
    const EdgeEntry& top = edge_queue_.top();
    if (top.version == top.parent->version && top.parent->queued_targets.count(top.child))
      return &top;
    edge_queue_.pop();  // stale: already processed, or g of the parent changed and a newer entry exists
  }
  return nullptr;
}

std::vector<BITStar::Vertex*> BITStar::neighbours(const Vertex* v, const bool in_tree) const
{
  const graph::core::NearestNeighborsPtr& nn = in_tree ? tree_nn_ : samples_;
  const Eigen::VectorXd& q = v->node->getConfiguration();

  // v is a tree vertex: in the tree index it is its own nearest neighbour, ask for one more
  std::multimap<double, NodePtr> near =
      knearest_ ? nn->kNearestNeighbors(q, k_neighbours_ + (in_tree ? 1 : 0)) : nn->near(q, radius_);

  std::vector<Vertex*> result;
  result.reserve(near.size());
  for (const auto& item : near)
  {
    Vertex* u = findVertex(item.second.get());
    if (u && u != v)
      result.push_back(u);
  }
  return result;
}

void BITStar::expandVertex(Vertex* v)
{
  const double c_best = bestCost();
  const Eigen::VectorXd& q = v->node->getConfiguration();

  // Edges towards the samples
  for (Vertex* x : neighbours(v, false))
  {
    if (v->invalid_targets.count(x->id))
      continue;
    const double c_hat = metrics_->utopia(q, x->node->getConfiguration());
    if (v->g_hat + c_hat + x->h_hat < c_best)
      queueEdge(v, x, c_hat);
  }

  // Rewiring edges towards the other vertices, only for the vertices added in this batch
  if (not v->is_new)
    return;

  const graph::core::Node* v_parent = parentOf(v->node.get());
  for (Vertex* w : neighbours(v, true))
  {
    if (w->node.get() == v_parent || parentOf(w->node.get()) == v->node.get())
      continue;  // already an edge of the tree
    if (v->invalid_targets.count(w->id))
      continue;
    const double c_hat = metrics_->utopia(q, w->node->getConfiguration());
    if (v->g_hat + c_hat + w->h_hat < c_best && v->g + c_hat < w->g)
      queueEdge(v, w, c_hat);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Edge processing
// ---------------------------------------------------------------------------------------------------------------

bool BITStar::processEdge(const EdgeEntry& edge)
{
  Vertex* v = edge.parent;
  Vertex* x = edge.child;
  const double c_best = bestCost();

  // The best edge cannot improve the solution: the batch is over
  if (v->g + edge.c_hat + x->h_hat >= c_best)
    return false;

  // Lazy removal of the edges that cannot improve the target anymore
  if (v->g + edge.c_hat >= x->g)
    return true;
  if (x->in_tree && parentOf(x->node.get()) == v->node.get())
    return true;

  // True edge cost first, the collision check only if the edge is still useful
  const double c = metrics_->cost(v->node->getConfiguration(), x->node->getConfiguration());
  if (not std::isfinite(c))
  {
    v->invalid_targets.insert(x->id);
    x->invalid_targets.insert(v->id);
    return true;
  }
  if (v->g_hat + c + x->h_hat >= c_best)
    return true;
  if (v->g + c >= x->g)
    return true;

  edges_checked_++;
  if (not checker_->checkConnection(v->node->getConfiguration(), x->node->getConfiguration()))
  {
    v->invalid_targets.insert(x->id);
    x->invalid_targets.insert(v->id);
    return true;
  }

  // With non-negative costs this cannot happen (a descendant has g >= g(x)), but a cycle would break the tree
  if (x->in_tree && isAncestor(x, v))
    return true;

  connect(v, x, c);
  return true;
}

void BITStar::connect(Vertex* parent, Vertex* child, const double& c)
{
  const bool was_in_tree = child->in_tree;
  if (was_in_tree)
  {
    child->node->parentConnection(0)->remove();
    rewirings_++;
  }
  else
  {
    samples_->deleteNode(child->node);
    start_tree_->addNode(child->node, false);
    tree_nn_->insert(child->node);
    child->in_tree = true;
    child->is_new = true;
  }

  ConnectionPtr conn = std::make_shared<graph::core::Connection>(parent->node, child->node, logger_);
  conn->setCost(c);
  conn->add();

  child->g = parent->g + c;
  child->version++;

  if (was_in_tree)
    propagateCost(child);  // re-queues child and its descendants
  else
    queueVertex(child);
}

void BITStar::propagateCost(Vertex* v)
{
  std::vector<Vertex*> stack{ v };
  while (not stack.empty())
  {
    Vertex* u = stack.back();
    stack.pop_back();

    // u->g and u->version are already updated: re-queue its entries with the new key
    if (u->in_vertex_queue)
      queueVertex(u);
    for (Vertex* target : std::vector<Vertex*>(u->queued_targets.begin(), u->queued_targets.end()))
      queueEdge(u, target, metrics_->utopia(u->node->getConfiguration(), target->node->getConfiguration()));

    for (const ConnectionPtr& conn : u->node->getChildConnections())
    {
      Vertex* child = findVertex(conn->getChild().get());
      child->g = u->g + conn->getCost();
      child->version++;
      stack.push_back(child);
    }
  }
}

bool BITStar::isAncestor(const Vertex* ancestor, const Vertex* v) const
{
  for (const graph::core::Node* n = v->node.get(); n; n = parentOf(n))
  {
    if (n == ancestor->node.get())
      return true;
  }
  return false;
}

bool BITStar::updateSolution()
{
  const double c = bestCost();
  if (not(c < path_cost_ - 1e-10 * std::max(1.0, std::abs(c))))
    return false;

  solution_ =
      std::make_shared<graph::core::Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
  solution_->setTree(start_tree_);

  path_cost_ = solution_->cost();
  cost_ = path_cost_ + goal_cost_;
  sampler_->setCost(path_cost_);
  solved_ = true;
  if (cost_ <= utopia_tolerance_ * best_utopia_)
    can_improve_ = false;

  GC_DEBUG(logger_, "BIT* solution improved: cost " << cost_ << " (path " << path_cost_ << "), batch " << batches_
                                                    << ", " << edges_checked_ << " edges checked");
  return true;
}

// ---------------------------------------------------------------------------------------------------------------
// Anytime interface
// ---------------------------------------------------------------------------------------------------------------

bool BITStar::update(graph::core::PathPtr& solution)
{
  solution = solution_;
  if (not problem_set_)
    return false;

  if (solved_ && cost_ <= utopia_tolerance_ * best_utopia_)
  {
    can_improve_ = false;
    return true;
  }

  if (not topVertex() && not topEdge())
  {
    newBatch();
    return false;
  }

  // Expand the vertices while the best vertex is not worse than the best edge. A vertex whose queue value is not
  // better than the current solution gives only edges that cannot improve it.
  const double c_best = bestCost();
  for (const VertexEntry* best_vertex = topVertex(); best_vertex; best_vertex = topVertex())
  {
    if (best_vertex->key >= c_best)
      break;
    const EdgeEntry* best_edge = topEdge();
    if (best_edge && best_vertex->key > best_edge->key)
      break;

    Vertex* v = best_vertex->vertex;
    vertex_queue_.pop();
    v->in_vertex_queue = false;
    expandVertex(v);
  }

  const EdgeEntry* best_edge = topEdge();
  if (not best_edge)
  {
    clearQueues();  // nothing left that can improve the solution: new batch at the next step
    return false;
  }

  const EdgeEntry edge = *best_edge;
  edge_queue_.pop();
  edge.parent->queued_targets.erase(edge.child);
  edges_processed_++;

  if (not processEdge(edge))
    clearQueues();

  if (updateSolution())
  {
    solution = solution_;
    return true;
  }
  return false;
}

bool BITStar::update(const Eigen::VectorXd& configuration, graph::core::PathPtr& solution)
{
  if (problem_set_ && configuration.size() == static_cast<Eigen::Index>(dof_))
  {
    const double f_hat = metrics_->utopia(start_tree_->getRoot()->getConfiguration(), configuration) +
                         metrics_->utopia(configuration, goal_node_->getConfiguration());
    if (f_hat < bestCost() && checker_->check(configuration))
    {
      NodePtr node = std::make_shared<graph::core::Node>(configuration, logger_);
      createVertex(node);
      samples_->insert(node);
    }
  }
  return update(solution);
}

bool BITStar::update(const NodePtr& n, graph::core::PathPtr& solution)
{
  return update(n->getConfiguration(), solution);
}

bool BITStar::solve(graph::core::PathPtr& solution, const unsigned int& max_iter, const double& max_time)
{
  if (not initialized_ || not problem_set_)
    return false;

  const graph::core::graph_time_point tic = graph::core::graph_time::now();
  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    update(solution);
    if (not can_improve_)
      break;
    if (graph::core::toSeconds(graph::core::graph_time::now(), tic) >= 0.98 * max_time)
      break;
  }

  solution = solution_;
  GC_DEBUG(logger_, "BIT* solve: solved " << solved_ << ", can improve " << can_improve_ << ", cost " << cost_
                                          << ", utopia " << best_utopia_ * utopia_tolerance_ << ", batches " << batches_
                                          << ", vertices " << tree_nn_->size() << ", samples " << samples_->size());
  return solved_;
}

void BITStar::printMyself(std::ostream& os) const
{
  TreeSolver::printMyself(os);
  os << "\nBIT*: batches " << batches_ << ", vertices " << (tree_nn_ ? tree_nn_->size() : 0) << ", samples "
     << getNumberOfSamples() << ", edges processed " << edges_processed_ << ", edges checked " << edges_checked_
     << ", rewirings " << rewirings_ << ", radius " << radius_;
}

}  // end namespace bit_star
}  // end namespace graph
