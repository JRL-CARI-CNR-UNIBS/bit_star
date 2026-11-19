# bit_star API reference

This page is generated from the Doxygen comments in the source code. To browse the same documentation as HTML,
run `doxygen Doxyfile` in the repository folder and open `doxygen/html/index.html`. The parameters and the
algorithm are described in the [README](../README.md).

## Contents

- [`include/bit_star/solvers/bit_star.h`](#includebit_starsolversbit_starh)
  - [`graph::bit_star::BITStar`](#class-graphbit_starbitstar)
- [`include/bit_star/plugins/solvers/bit_star_plugin.h`](#includebit_starpluginssolversbit_star_pluginh)
  - [`graph::bit_star::BITStarPlugin`](#class-graphbit_starbitstarplugin)

## include/bit_star/solvers/bit_star.h

Batch Informed Trees (BIT\*) solver.

### class `graph::bit_star::BITStar`

Inherits from `graph::core::TreeSolver`.

BIT\* (Gammell, Srinivasa, Barfoot, ICRA 2015 / IJRR 2020) is an anytime, asymptotically optimal planner. It
samples the configuration space in batches of collision-free states, which together with the tree vertices form
an implicit random geometric graph (RGG). The RGG is searched in order of the estimated solution cost through it,
as in A\*: the edges are processed from a queue sorted by g_T(v) + ĉ(v,x) + ĥ(x), where g_T is the cost-to-come in
the tree, ĉ an admissible estimate of the edge cost (`MetricsBase::utopia()`) and ĥ an admissible estimate of the
cost-to-go. The true edge cost and the collision check are computed only when an edge is taken from the queue.
When the queue holds no edge that can improve the current solution the batch ends: the samples and vertices that
cannot improve the solution are pruned, a new batch is drawn from the informed set, and the search restarts from
the current tree.

Integration with graph_core:

- the tree is a regular `graph::core::Tree` (`start_tree_`), so the solution is a regular `graph::core::Path`;
- the unconnected samples live in a separate `graph::core::KdTree`, and the tree vertices are indexed in a
  second one (`Tree::nearK()` uses its own number of neighbours);
- the per-vertex data needed by the search (cached g_T, ĝ, ĥ, queue bookkeeping) are kept in a side table indexed
  by node, not in the graph_core nodes.

One call to `update()` performs one step of the search: it expands the vertices whose queue value is not worse
than the best edge, then processes the best edge. When both queues are empty, `update()` draws a new batch
instead. `solve()` and `computePath()` therefore need a large `max_iter` (one iteration ≈ one edge).

The sampler defines the informed set: use a `graph::core::InformedSampler` whose foci are start and goal, BIT\*
calls `SamplerBase::setCost()` with the path cost at each improvement.

Parameters read by `config()` (besides those of `graph::core::TreeSolver`):

| Name | Type | Default | Description |
|---|---|---|---|
| `samples_per_batch` | int | 100 | number of collision-free samples drawn at each batch |
| `sample_attempts_factor` | int | 10 | at most `sample_attempts_factor * samples_per_batch` draws per batch |
| `rewire_factor` | double | 1.1 | η, multiplies the RGG radius (or the number of neighbours) |
| `knearest` | bool | false | use the k-nearest RGG instead of the r-disc RGG |
| `prune` | bool | true | prune samples and vertices that cannot improve the solution |
| `prune_threshold` | double | 0.05 | prune only if the cost decreased by this fraction since the last pruning |
| `direct_connection` | bool | true | try the edge start-goal when the problem is set, as TreeSolver does |

`max_distance` and `extend` of `TreeSolver` are not used: BIT\* connects samples with straight edges and never
steers.

#### Public functions

##### `BITStar`

```cpp
BITStar(const graph::core::MetricsPtr& metrics, const graph::core::CollisionCheckerPtr& checker,
        const graph::core::SamplerPtr& sampler, const graph::core::GoalCostFunctionPtr& goal_cost_fcn,
        const graph::core::LoggerPtr& logger)
BITStar(const graph::core::MetricsPtr& metrics, const graph::core::CollisionCheckerPtr& checker,
        const graph::core::SamplerPtr& sampler, const graph::core::LoggerPtr& logger)
```

Constructors. Without `goal_cost_fcn`, the goal has no cost. `config()` must be called before adding start and
goal.

##### `config`

```cpp
virtual bool config(const graph::core::ParamMap& params) override
```

Reads the parameters (see the class description) and resets the solver status.

**Returns:** false if the `TreeSolver` configuration fails.

##### `update`

```cpp
virtual bool update(graph::core::PathPtr& solution) override
```

One step of the search (see the class description).

**Parameters:**

- `solution`: set to the current solution (nullptr if not solved yet).

**Returns:** true if the solution was found or improved in this step, or if it is already optimal within
`utopia_tolerance`.

```cpp
virtual bool update(const Eigen::VectorXd& configuration, graph::core::PathPtr& solution) override
virtual bool update(const graph::core::NodePtr& n, graph::core::PathPtr& solution) override
```

Adds a configuration to the samples (if it is collision-free and can improve the solution), then performs one
step of the search.

##### `solve`

```cpp
virtual bool solve(graph::core::PathPtr& solution, const unsigned int& max_iter = 100,
                   const double& max_time = std::numeric_limits<double>::infinity()) override
```

Runs `update()` until the solution is optimal within `utopia_tolerance`, `max_iter` steps are done, or
`max_time` is elapsed. Unlike `TreeSolver::solve()`, it keeps improving the solution after the first one.

**Returns:** true if a solution exists.

##### `addStart`

```cpp
virtual bool addStart(const graph::core::NodePtr& start_node,
                      const double& max_time = std::numeric_limits<double>::infinity()) override
```

Creates the tree rooted at `start_node`. The start must be in bounds and collision-free.

##### `addGoal`

```cpp
virtual bool addGoal(const graph::core::NodePtr& goal_node,
                     const double& max_time = std::numeric_limits<double>::infinity()) override
```

Sets the goal. The goal must be in bounds and collision-free.

##### `addStartTree`

```cpp
virtual bool addStartTree(const graph::core::TreePtr& start_tree,
                          const double& max_time = std::numeric_limits<double>::infinity()) override
```

Starts from an existing tree (e.g. from a previous query). The connections of the tree are assumed collision-free
and their cost up to date.

##### `resetProblem`

```cpp
virtual void resetProblem() override
```

Removes start, goal, tree and search data.

##### `setSolution`

```cpp
virtual bool setSolution(const graph::core::PathPtr& solution) override
```

Sets the solution and its tree, and rebuilds the search data from that tree.

##### `importFromSolver`

```cpp
virtual bool importFromSolver(const graph::core::TreeSolverPtr& solver) override
```

Imports the problem from another solver and rebuilds the search data from its tree.

##### `getCostToCome`

```cpp
double getCostToCome(const graph::core::NodePtr& node) const
```

Cost-to-come g_T cached by the solver, infinity if the node is not a tree vertex.

##### Statistics

| Function | Returns |
|---|---|
| `unsigned int getNumberOfSamples() const` | number of unconnected samples |
| `unsigned int getNumberOfBatches() const` | number of batches drawn since the problem was set |
| `unsigned long getNumberOfProcessedEdges() const` | edges taken from the edge queue |
| `unsigned long getNumberOfCheckedEdges() const` | edges collision-checked |
| `unsigned long getNumberOfRewirings() const` | times a tree vertex got a better parent |
| `double getRadius() const` | current radius of the r-disc RGG (infinity before the first batch) |
| `unsigned int getKNeighbours() const` | current number of neighbours of the k-nearest RGG |

#### Protected types

| Type | Description |
|---|---|
| `Vertex` | search data of a tree vertex or sample: `node`, `id` (never reused), `g_hat` (ĝ), `h_hat` (ĥ), `g` (g_T, infinity for samples), `in_tree`, `is_new` (added in the current batch), `in_vertex_queue`, `version` (incremented when `g` changes), `queued_targets`, `invalid_targets`; `fHat()` = ĝ + ĥ |
| `VertexEntry` | vertex queue entry: key g_T + ĥ, tie-break g_T, vertex, version |
| `EdgeEntry` | edge queue entry: key g_T(v) + ĉ + ĥ(x), tie-breaks g_T(v) + ĉ and g_T(v), ĉ, parent, child, parent version |
| `VertexEntryGreater`, `EdgeEntryGreater` | min-heap orders |

#### Protected functions

| Function | Description |
|---|---|
| `setProblem(max_time)` | initialises the search state (start vertex and goal sample) and, with `direct_connection`, tries the edge start-goal |
| `clearSearch()` | removes all the search data (vertices, samples, queues and statistics) |
| `initSearchFromTree()` | rebuilds the search data from `start_tree_`: every tree node becomes a vertex with g = cost-to-come, and the goal becomes a sample if it is not in the tree |
| `createVertex(node)`, `findVertex(node)` | search data of a node |
| `bestCost()` | path cost of the current solution (g_T of the goal), infinity if not solved |
| `newBatch()` | ends the current batch: prunes (if enabled and worth it), draws a new batch, queues all the vertices, updates the RGG radius |
| `sampleBatch()` | draws `samples_per_batch` collision-free samples that can improve the solution; returns the number added |
| `prune(cost)` | removes the samples with f̂ ≥ cost and the vertices with f̂ > cost, recycling as samples the descendants that can still improve the solution; returns the number of nodes removed from the tree |
| `updateRadius()` | updates the radius and k from the number of states and the measure of the informed set |
| `informedSpecificVolume()` | λ(X_f̂)/ζ_n: minimum of the bounding box and `SamplerBase::getSpecificVolume()` |
| `neighbours(v, in_tree)` | neighbours of a vertex among the samples or the tree vertices |
| `expandVertex(v)` | queues the outgoing edges of a vertex |
| `queueEdge(parent, child, c_hat)`, `queueVertex(v)`, `clearQueues()` | queue operations |
| `topVertex()`, `topEdge()` | pop the stale entries and return the best valid one, or nullptr |
| `processEdge(edge)` | true cost, collision check and connection; returns false if the batch is over |
| `connect(parent, child, c)` | adds or replaces the parent edge of child and propagates the new cost-to-come |
| `propagateCost(v)` | recomputes g of the descendants of v and re-queues their entries |
| `isAncestor(ancestor, v)` | true if `ancestor` is v or one of its ancestors |
| `updateSolution()` | updates the solution, the costs and the sampler if the goal cost-to-come improved |

### Free functions and types

#### `BITStarPtr`

```cpp
typedef std::shared_ptr<BITStar> BITStarPtr
```

## include/bit_star/plugins/solvers/bit_star_plugin.h

cnr_class_loader plugin that creates a `graph::bit_star::BITStar`.

### class `graph::bit_star::BITStarPlugin`

Inherits from `graph::core::TreeSolverPlugin`.

Exposes `graph::bit_star::BITStar` as a `graph::core::TreeSolverPlugin`, loadable by name with cnr_class_loader
(class name `graph::bit_star::BITStarPlugin`, library `libbit_star.so`).

#### Public functions

##### `BITStarPlugin`

```cpp
BITStarPlugin()
```

Empty constructor; `init()` must be called afterwards.

##### `init`

```cpp
virtual bool init(const graph::core::ParamMap& params, const graph::core::MetricsPtr& metrics,
                  const graph::core::CollisionCheckerPtr& checker, const graph::core::SamplerPtr& sampler,
                  const graph::core::GoalCostFunctionPtr& goal_cost_fcn,
                  const graph::core::LoggerPtr& logger) override
```

Creates the solver. The parameters are not read here: as for the other graph_core solvers, they are passed later
to `TreeSolver::config()` (or to `TreeSolver::computePath()`).

**Returns:** true.
