# Tutorial — Planning with BIT\* in graph_core

This tutorial shows how to plan with `graph::bit_star::BITStar`: first with the example shipped with the
package, then with your own problem, as a plugin, and when replanning. No robot or ROS installation is needed.

1. [Build](#0-build) (5 min)
2. [Run the example](#1-run-the-example) (5 min)
3. [Write your own problem](#2-write-your-own-problem) (15 min)
4. [Use BIT\* as a plugin](#3-use-bit-as-a-plugin) (5 min)
5. [Replan from a previous tree](#4-replan-from-a-previous-tree) (5 min)
6. [What to check in the log](#5-what-to-check-in-the-log)
7. [Parameter tuning and common problems](#6-parameter-tuning-and-common-problems)

---

## 0. Build

In a colcon workspace that already contains (or can fetch) graph_core and cnr_class_loader:

```bash
cd ~/ros2_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/bit_star.git
vcs import < bit_star/deps.repos   # skip if graph_core and cnr_class_loader are already there
cd ~/ros2_ws
colcon build --packages-up-to bit_star
source install/setup.bash
colcon test --packages-select bit_star && colcon test-result --verbose
```

graph_core must be on the branch `removing_cnr_logger_and_params` (with `graph::core::ParamMap`).

---

## 1. Run the example

`bit_star_wall_example` plans in the square [-1.5, 1.5]² from (-1, 0.8) to (1, 0.8). A vertical wall
|x| ≤ 0.05 blocks the way, except for a slit |y| < 0.1. The optimal path touches the two corners of the slit
and costs 2·√(0.95² + 0.7²) + 0.1 ≈ 2.4601.

```bash
~/ros2_ws/install/bit_star/lib/bit_star/bit_star_wall_example 2 info
```

Expected output (the numbers change at every run, since the samples are random):

```
[INFO] [bit_star_wall_example] ... t = 0.0004 s, cost 2.53464, batch 1, 21 vertices, 81 samples, 69 edges checked
[INFO] [bit_star_wall_example] ... t = 0.0013 s, cost 2.52159, batch 2, 63 vertices, 69 samples, 255 edges checked
...
[INFO] [bit_star_wall_example] ... t = 1.9075 s, cost 2.46139, batch 257, 13118 vertices, 12146 samples, 20486 edges checked
[INFO] [bit_star_wall_example] ... Final cost 2.46139 (optimum 2.46008, +0.05 %)
```

followed by the waypoints of the path. Things to notice:

- the first solution comes from the very first batch: BIT\* searches the 100 samples in order of the estimated
  solution cost, so the first path found is already a good one;
- only a fraction of the possible edges is ever collision-checked;
- the cost never increases.

Run it with `debug` instead of `info` to see one line per batch (samples drawn, vertices pruned, radius).

---

## 2. Write your own problem

The example (`examples/wall_example.cpp`) contains all the steps. The parts that change from problem to problem:

### 2.1 Collision checker

Derive from `graph::core::CollisionCheckerBase` and implement `check()` (true = collision-free). The base class
checks an edge by sampling it every `min_distance`:

```cpp
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
```

With a robot, use the MoveIt checker of `moveit_collision_checker` instead.

### 2.2 Metrics and sampler

```cpp
graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);
graph::core::SamplerPtr sampler = std::make_shared<graph::core::InformedSampler>(start, goal, lb, ub, logger);
```

BIT\* uses `metrics->utopia()` as the admissible estimate of the edge cost, of the cost-to-come and of the
cost-to-go, and `metrics->cost()` as the true edge cost. With `EuclideanMetrics` they coincide.

Give the `InformedSampler` the start and goal as foci: BIT\* calls `sampler->setCost()` at every improvement,
so the new samples are drawn only where they can still improve the path. A `UniformSampler` also works, but the
samples outside the informed set are drawn and then discarded.

### 2.3 Parameters

graph_core does not read files or parameter servers: the application fills a `graph::core::ParamMap`
(in a ROS node, from the node parameters; from a YAML file with `graph::core::paramsFromYAML()`):

```cpp
graph::core::ParamMap params;
params["samples_per_batch"] = int64_t(100);  // integers are int64_t
params["rewire_factor"] = 1.1;
params["prune_threshold"] = 0.05;
params["utopia_tolerance"] = 0.001;
```

Missing parameters take their default, with a warning. The full list is in the README.

### 2.4 Solve

Either in one call:

```cpp
graph::bit_star::BITStar solver(metrics, checker, sampler, logger);
graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start, logger);
graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal, logger);

graph::core::PathPtr solution;
bool found = solver.computePath(start_node, goal_node, params, solution, /*max_time=*/2.0,
                                /*max_iter=*/100000000);
```

or step by step, to stop on your own condition or to show the progress:

```cpp
solver.config(params);
solver.addStart(start_node);   // false if out of bounds or in collision
solver.addGoal(goal_node);

graph::core::PathPtr solution;
while (not done && solver.canImprove())
{
  if (solver.update(solution))  // true when the solution is found or improved
    std::cout << "cost " << solver.getCost() << std::endl;
}
```

One `update()` is one step of the search (about one edge, or the drawing of a batch), so `max_iter` must be large;
use `max_time` to bound the planning time.

### 2.5 Read the result

`solution` is a regular `graph::core::Path`: `cost()`, `getWaypoints()`, `getConnections()`, and the
graph_core post-processing (e.g. `graph::core::PathLocalOptimizer`) work as with the other solvers.
`solver.getStartTree()` is the search tree.

---

## 3. Use BIT\* as a plugin

Applications that load the solver by name (e.g. through a `solver_plugin` parameter) only need the plugin name:

```cpp
#include <cnr_class_loader/multi_library_class_loader.hpp>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>

cnr_class_loader::MultiLibraryClassLoader loader(false);
loader.loadLibrary("libbit_star.so");
auto plugin = loader.createInstance<graph::core::TreeSolverPlugin>("graph::bit_star::BITStarPlugin");
plugin->init(params, metrics, checker, sampler, goal_cost_fcn, logger);
graph::core::TreeSolverPtr solver = plugin->getSolver();
```

`cnr_class_loader` searches a library name in the folders of `LD_LIBRARY_PATH` (set by
`source install/setup.bash`) and in `/usr/lib`, `/usr/local/lib`; an absolute path is loaded as it is (with the
cnr_class_loader fix on branch `fix/library-path-lookup`; before it, only names worked). Keep the loader alive
while you use the solver, and release the solver before the loader.

To compare BIT\* with RRT\* on your machine, run
`~/ros2_ws/install/bit_star/lib/bit_star/bit_star_compare_rrt_star --runs 3 twocubes3d` (50 s per run and solver;
`cube<n>d` and `twocubes<n>d` accept any dimension; see the README for the problems and the results).

---

## 4. Replan from a previous tree

The tree of a previous query can seed a new one with the same start:

```cpp
graph::bit_star::BITStar second(metrics, checker, sampler, logger);
second.config(params);
second.addStartTree(solution->getTree());  // its connections must still be collision-free
second.addGoal(goal_node);                 // already solved if the goal is in the tree
second.solve(solution, 100000000, 1.0);
```

If the environment changed, recheck the tree first (`Tree::recheckCollision()`).

---

## 5. What to check in the log

With the `debug` level, every batch prints

```
BIT* batch 12: 100 new samples, 37 vertices pruned, 412 vertices, 830 samples, radius 0.21, k 0, cost 2.47
```

- **new samples** lower than `samples_per_batch`: most draws were in collision or outside the informed set,
  and the attempts (`sample_attempts_factor`) ran out;
- **vertices pruned** > 0 only after an improvement larger than `prune_threshold`;
- **radius** shrinks with the number of states and with the informed set; if it becomes much smaller than the
  narrow passages of your problem, the graph cannot go through them until more samples are drawn there.

---

## 6. Parameter tuning and common problems

| Symptom | Cause and fix |
|---|---|
| `addStart()`/`addGoal()` return false | start or goal out of the sampler bounds, or in collision |
| No solution in narrow passages | increase `samples_per_batch` (denser graph per batch) or `rewire_factor`; raise `sample_attempts_factor` if most of the space is in collision |
| First batch takes long | too many `samples_per_batch` for an expensive collision checker: lower it |
| Slow improvement after the first solution | lower `prune_threshold` so that the states outside the informed set are pruned (and the radius computed on the right density) more often |
| Path in free space is a polyline | `direct_connection: false` and the goal farther than the radius: expected, it converges to the straight line over the batches |
| Memory grows | `prune: false`, or the cost improves less than `prune_threshold` for a long time |
| Stops before `max_time` | the cost is within `utopia_tolerance` of the utopia: lower the tolerance for near-optimal paths |
