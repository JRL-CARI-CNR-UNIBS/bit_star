# bit_star

[![build-and-test](https://github.com/JRL-CARI-CNR-UNIBS/bit_star/actions/workflows/build_and_test.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/bit_star/actions/workflows/build_and_test.yaml)
[![clang-format](https://github.com/JRL-CARI-CNR-UNIBS/bit_star/actions/workflows/clang-format.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/bit_star/actions/workflows/clang-format.yaml)

**Batch Informed Trees (BIT\*)** path planner for
[graph_core](https://github.com/JRL-CARI-CNR-UNIBS/graph_core).

BIT\* is an anytime, asymptotically optimal sampling-based planner (Gammell, Srinivasa, Barfoot, ICRA 2015 and
IJRR 2020). It draws batches of collision-free samples, which together with the tree form an implicit random
geometric graph (RGG), and searches that graph in order of the estimated cost of the solution, like A\*. Edges are
collision-checked only when the search reaches them. When a solution exists, the next batches are drawn only in
the informed set (the states that can still improve it), and the samples and vertices outside it are pruned.

`graph::bit_star::BITStar` is a `graph::core::TreeSolver`: it takes the usual graph_core metrics, collision
checker, sampler and goal cost function, and returns a `graph::core::Path` on a `graph::core::Tree`. It is also
exported as the `cnr_class_loader` plugin `graph::bit_star::BITStarPlugin`, so it can replace RRT\* (or MI-RRT\*,
[mirrt_star](https://github.com/JRL-CARI-CNR-UNIBS/mirrt_star)) in applications that load solvers by name.
The library does not depend on ROS.

See the **[tutorial](docs/TUTORIAL.md)** for a step-by-step example.

## Requirements

- Ubuntu 22.04 or 24.04, C++20 compiler, CMake ≥ 3.16
- [graph_core](https://github.com/JRL-CARI-CNR-UNIBS/graph_core), branch `removing_cnr_logger_and_params`
  (the one with `graph::core::ParamMap` and the abstract logger, and the include guard of
  `datastructure/vector.h`), and its dependencies: Eigen3, yaml-cpp,
  [cnr_class_loader](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader) (branch `modern_cmake`)
- GoogleTest for the tests (`libgtest-dev`)

## Build

In a colcon workspace (ROS 2 is not required, colcon just builds the CMake packages in order):

```bash
cd ~/ros2_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/bit_star.git
vcs import < bit_star/deps.repos        # graph_core and cnr_class_loader, if not already in the workspace
cd ~/ros2_ws
colcon build --packages-up-to bit_star
source install/setup.bash
```

As a plain CMake package (graph_core must be installed and findable by CMake):

```bash
cmake -S bit_star -B build/bit_star -DCMAKE_INSTALL_PREFIX=<install_dir> -DCMAKE_PREFIX_PATH=<install_dir>
cmake --build build/bit_star --target install
```

Then, in your CMake project:

```cmake
find_package(bit_star REQUIRED)
target_link_libraries(my_target PUBLIC bit_star::bit_star)
```

CMake options: `BUILD_TESTS` (default `ON`), `BUILD_EXAMPLES` (default `ON`). The `format` target runs
clang-format on all the sources.

## Running

The example plans in the plane, through a narrow slit in a wall, and prints the cost at each improvement:

```bash
# colcon: install/bit_star/lib/bit_star/bit_star_wall_example; plain CMake: build/bit_star/bit_star_wall_example
bit_star_wall_example 2 info
```

| Argument | Default | Description |
|---|---|---|
| `max_time_s` | `2` | planning time [s] |
| `log_level` | `info` | `trace`, `debug`, `info`, `warn`, `error`, `fatal` (`debug` prints one line per batch) |

Typical output: first solution after ~1 ms and ~70 collision-checked edges, within 0.5 % of the optimum after
~1 s.

Minimal use in C++:

```cpp
#include <bit_star/solvers/bit_star.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/samplers/informed_sampler.h>

auto metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);
// foci on start and goal: BIT* shrinks the informed set with SamplerBase::setCost()
auto sampler = std::make_shared<graph::core::InformedSampler>(start, goal, lb, ub, logger);
auto solver = std::make_shared<graph::bit_star::BITStar>(metrics, checker, sampler, logger);

graph::core::ParamMap params;
params["samples_per_batch"] = int64_t(100);

graph::core::PathPtr solution;
bool found = solver->computePath(start, goal, params, solution, /*max_time=*/1.0, /*max_iter=*/100000000);
```

`computePath()` keeps improving the solution until `max_time`, `max_iter` or the utopia tolerance is reached.
One iteration is one step of the search (about one edge), so give a large `max_iter`.

As a plugin:

```cpp
#include <cnr_class_loader/multi_library_class_loader.hpp>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>

cnr_class_loader::MultiLibraryClassLoader loader(false);
loader.loadLibrary("libbit_star.so");  // a name is searched in the LD_LIBRARY_PATH folders; an absolute path
                                        // is loaded as it is (cnr_class_loader branch fix/library-path-lookup)
auto plugin = loader.createInstance<graph::core::TreeSolverPlugin>("graph::bit_star::BITStarPlugin");
plugin->init(params, metrics, checker, sampler, goal_cost_fcn, logger);
graph::core::TreeSolverPtr solver = plugin->getSolver();
```

## Comparison with RRT\*

`bit_star_compare_rrt_star [runs] [max_distance]` runs BIT\* and graph_core's `RRTStar` on the same problems,
with the same `InformedSampler`, Euclidean metrics, collision checker and parameters (`max_distance` 1.0,
`rewire_factor` 1.1, `samples_per_batch` 100). Median cost over 7 runs (worst run in brackets), Release build,
desktop CPU:

| Problem (optimum) | Solver | 0.05 s | 0.1 s | 0.25 s | 0.5 s | 1 s |
|---|---|---|---|---|---|---|
| wall with slit, 2D (2.4601) | BIT\* | 2.4775 [2.4825] | 2.4757 [2.4811] | 2.4718 [2.4766] | 2.4676 [2.4749] | 2.4656 [2.4745] |
| | RRT\* | 2.4837 [2.4931] | 2.4782 [2.4894] | 2.4750 [2.4804] | 2.4702 [2.4767] | 2.4690 [2.4749] |
| cube obstacle, 3D (5.9161) | BIT\* | 6.0129 [6.0230] | 6.0045 [6.0201] | 5.9831 [5.9976] | 5.9800 [5.9976] | 5.9746 [5.9855] |
| | RRT\* | 6.1704 [6.1795] | 6.0968 [6.1705] | 6.0789 [6.1161] | 6.0658 [6.1095] | 6.0615 [6.0827] |

Both find the first solution in less than 1 ms. In 3D BIT\* is better at every instant: after 0.1 s it is below
the cost RRT\* reaches in 1 s (+1.0 % from the optimum against +2.5 %). In 2D the advantage is smaller and
shrinks with time (at 1 s the worst runs are equal). The optimum of the cube problem is the shortest path bending
once on an edge of the cube (√35). The test `BeatsRRTStarOnCubeObstacle` checks the 3D result (median of 5 runs
at 0.25 s). The samplers are not seeded, so the numbers change slightly at every run.

## Algorithm

```
          ┌──────────────── new batch ────────────────┐
          │ prune samples/vertices with f̂ ≥ c_best    │
          │ draw m collision-free samples, f̂ < c_best │
          │ Q_V = all vertices,  r = r(q, λ(X_f̂))     │
          └─────────────────────┬─────────────────────┘
                                │
          ┌─────────────────────▼─────────────────────┐
          │ while best(Q_V) ≤ best(Q_E):              │
          │   expand v: queue (v,x), x within r       │
          ├───────────────────────────────────────────┤
          │ pop best edge (v,x) from Q_E              │
          │ g_T(v)+ĉ+ĥ(x) ≥ c_best ? ──► batch over ──┼──► new batch
          │ true cost c, then collision check         │
          │ g_T(v)+c < g_T(x) ? ──► connect / rewire  │
          └─────────────────────┬─────────────────────┘
                                └──► goal improved ? update solution, sampler cost
```

Notation: g_T(x) cost-to-come in the tree, ĝ(x) = `utopia(start, x)`, ĥ(x) = `utopia(x, goal)`,
f̂ = ĝ + ĥ, ĉ(v,x) = `utopia(v, x)`, c(v,x) = `cost(v, x)` (all from `graph::core::MetricsBase`),
c_best = g_T(goal), the path cost of the current solution.

1. **Problem set.** The start is the root of the tree, the goal is the first sample. With `direct_connection`
   the edge start-goal is checked right away (as `TreeSolver::setProblem()` does); without it, the goal is
   reached only through the RGG.
2. **New batch** (when both queues are empty). If the cost decreased by more than `prune_threshold` since the
   last pruning, the samples with f̂ ≥ c_best and the vertices with f̂ > c_best are removed; the descendants of
   a removed vertex that can still improve the solution go back to the samples. Then `samples_per_batch`
   collision-free samples with f̂ < c_best are drawn (at most `sample_attempts_factor × samples_per_batch`
   draws), every vertex goes in the vertex queue and the RGG size is updated with q = vertices + samples:

   r = η · 2 (1 + 1/n)^(1/n) · (λ(X_f̂)/ζ_n)^(1/n) · (log q / q)^(1/n),  k = ⌈η · e (1 + 1/n) log q⌉

   λ(X_f̂)/ζ_n is the measure of the informed set divided by that of the unit n-ball: the minimum between
   the bounding box and `SamplerBase::getSpecificVolume()` (the ellipsoid, for an `InformedSampler` with
   finite cost).
3. **Vertex expansion.** While the best vertex (key g_T + ĥ) is not worse than the best edge (key
   g_T(v) + ĉ + ĥ(x)), it is expanded: the edges to the samples within r (or the k nearest) with
   ĝ(v) + ĉ + ĥ(x) < c_best are queued. Vertices added during the current batch also queue the edges to the
   other vertices that would lower their cost-to-come (rewiring).
4. **Edge processing.** The best edge is popped. If g_T(v) + ĉ + ĥ(x) ≥ c_best, no edge left can improve the
   solution: the queues are emptied and the next step starts a new batch. Otherwise the true cost is computed,
   and only if the edge can still improve both the solution and g_T(x) is it collision-checked. A valid edge
   connects a sample to the tree, or replaces the parent of a vertex; in that case the new cost-to-come is
   propagated to the descendants.
5. **Solution.** When g_T(goal) decreases, the solution path is rebuilt and `SamplerBase::setCost()` shrinks
   the informed set. The search stops when the cost is within `utopia_tolerance` of the utopia (`canImprove()`
   becomes false).

Implementation notes:

- **Lazy queue keys.** Both queues are binary heaps. Each vertex has a version number, incremented when its
  g_T changes; entries with an old version are skipped when they reach the top, and a rewiring re-pushes the
  queued entries of the vertex and of its descendants with the new keys. Edges that can no longer improve their
  target are dropped when popped instead of being searched in the queue.
- **Cached cost-to-come.** g_T is stored per vertex and updated on rewiring, instead of calling
  `Tree::costToNode()` (which walks up to the root) at every queue operation.
- **Invalid edges** are remembered per vertex and never checked twice, also across batches.
- **Nearest neighbours.** Samples and tree vertices are indexed in two `graph::core::KdTree`s (or
  `graph::core::Vector` with `use_kdtree: false`). The samples index is rebuilt at every batch, since the
  `KdTree` only marks deleted nodes.

Options of the OMPL implementation (`BITstar`) that are not available here: delaying the rewiring until the
first solution, just-in-time sampling, dropping the unconnected samples at each pruning, stopping at each
solution improvement, and approximate solutions. They are listed under "Planned" in the
[changelog](CHANGELOG.md).

## Interfaces

`graph::bit_star::BITStar` implements the `graph::core::TreeSolver` interface:

| Method | Description |
|---|---|
| `config(params)` | reads the parameters, see below |
| `addStart(node)`, `addGoal(node)` | set start and goal; they must be in bounds and collision-free |
| `addStartTree(tree)` | start from an existing tree (e.g. from a previous query): its costs and connections are reused |
| `update(solution)` | one step of the search; true if the solution was found or improved |
| `update(configuration, solution)` | adds a user sample, then one step |
| `solve(solution, max_iter, max_time)` | anytime loop; true if a solution exists |
| `computePath(start, goal, params, solution, max_time, max_iter)` | `resetProblem()` + `config()` + `addStart()` + `addGoal()` + `solve()` |
| `setSolution(path)`, `importFromSolver(solver)` | warm start from a path or another solver, the search data are rebuilt from the tree |

Diagnostics: `getCostToCome(node)`, `getNumberOfBatches()`, `getNumberOfSamples()`,
`getNumberOfProcessedEdges()`, `getNumberOfCheckedEdges()`, `getNumberOfRewirings()`, `getRadius()`,
`getKNeighbours()`. Debug logging prints one line per batch and one per improvement.

The class is not thread-safe: call it from one thread.

## Parameters

Parameters are passed in a `graph::core::ParamMap` to `config()` or `computePath()`; a missing parameter takes
its default (with a warning in the log). Costs are in the units of the metrics.

| Parameter | Default | Description |
|---|---|---|
| `samples_per_batch` | `100` | collision-free samples drawn at each batch |
| `sample_attempts_factor` | `10` | at most `sample_attempts_factor × samples_per_batch` draws per batch (bounds the time spent in nearly-full spaces) |
| `rewire_factor` | `1.1` | η: scales the RGG radius or the number of neighbours; < 1 loses asymptotic optimality |
| `knearest` | `false` | k-nearest RGG instead of r-disc |
| `prune` | `true` | prune the samples and vertices that cannot improve the solution |
| `prune_threshold` | `0.05` | prune only if the cost decreased by this fraction since the last pruning; `0` prunes at every batch after an improvement |
| `direct_connection` | `true` | check the edge start-goal when the problem is set |
| `utopia_tolerance` | `0.01` | (TreeSolver) stop when cost ≤ (1 + tolerance) × utopia |
| `use_kdtree` | `true` | (TreeSolver) KdTree instead of a plain vector for the nearest-neighbour queries |
| `max_distance`, `extend` | `1.0`, `false` | (TreeSolver) not used by BIT\*, which never steers |

Tuning hints:

- **Larger `samples_per_batch`** gives a denser graph per batch and better first solutions in cluttered spaces, at
  the cost of a longer first batch. 50–200 is a good range up to 7 DOF.
- **Narrow passages:** keep `prune: true` and the `InformedSampler`; raise `sample_attempts_factor` if most of
  the space is in collision.
- **Slow convergence after the first solution:** lower `prune_threshold` (the RGG radius uses the number of states,
  which includes the ones outside the informed set until they are pruned).

## Package layout

```
bit_star/
├── CMakeLists.txt                              library, plugin, example, tests, install and export
├── package.xml                                 colcon manifest (build type cmake)
├── cmake_config/
│   ├── Dependencies.cmake                      find_package(graph_core)
│   ├── bit_starConfig.cmake.in                 config for find_package(bit_star)
│   └── clang_format_target.sh                  used by the `format` target
├── include/bit_star/
│   ├── solvers/bit_star.h                      graph::bit_star::BITStar
│   └── plugins/solvers/bit_star_plugin.h       graph::bit_star::BITStarPlugin
├── src/bit_star/
│   ├── solvers/bit_star.cpp                    BIT* implementation
│   └── plugins/solvers/bit_star_plugin.cpp     plugin registration
├── examples/
│   ├── wall_example.cpp                        wall with a slit in the plane
│   └── compare_rrt_star.cpp                    anytime comparison with RRT*
├── tests/
│   ├── CMakeLists.txt                          gtest suite
│   └── src/test_bit_star.cpp
├── docs/
│   ├── TUTORIAL.md                             step-by-step tutorial
│   └── API.md                                  API reference (from the Doxygen comments)
├── deps.repos                                  graph_core and cnr_class_loader for vcs import
├── .clang-format, .pre-commit-config.yaml, hooks/   formatting (same style as graph_core)
└── .github/workflows/                          CI: build and test, clang-format check
```

## Tests

```bash
colcon test --packages-select bit_star && colcon test-result --verbose
# or, in a plain CMake build folder
ctest --output-on-failure
```

The suite (13 tests, about 13 s) checks: straight line in free space (with and without direct connection),
near-optimal cost through the slit, a 3D cube obstacle, lower median cost than RRT\* on the cube, the k-nearest
RGG, monotonic cost, consistency of the cached g_T with the tree, the pruning invariant (no vertex with
f̂ > c_best after a pruning), reuse of a start tree, rejection of start and goal in collision, and loading through
the plugin. It needs no display, network or hardware.

## Authors

Developed by **CARI-JRL** (JRL-CARI CNR-STIIMA/UNIBS).
Maintainer: Manuel Beschi <manuel.beschi@unibs.it>.

## License

BSD 3-Clause, see [LICENSE](LICENSE). Contributions: see [CONTRIBUTING.md](CONTRIBUTING.md) and the
[code of conduct](CODE_OF_CONDUCT.md).

## API documentation

See [docs/API.md](docs/API.md), or generate the HTML documentation with `doxygen Doxyfile` and open
`doxygen/html/index.html`.

## References

- J. D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Batch Informed Trees (BIT\*): Sampling-based optimal planning
  via the heuristically guided search of implicit random geometric graphs", ICRA 2015.
- J. D. Gammell, T. D. Barfoot, S. S. Srinivasa, "Batch Informed Trees (BIT\*): Informed asymptotically optimal
  anytime search", IJRR 39(5), 2020.
