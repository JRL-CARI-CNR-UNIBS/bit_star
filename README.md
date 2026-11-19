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

## Comparison with RRT\* and MI-RRT\*

`bit_star_compare_rrt_star [--runs N] [--max-distance D] [--solver bit,rrt,mirrt] [problem ...]` runs BIT\*, graph_core's `RRTStar` and, if the
package [mirrt_star](https://github.com/JRL-CARI-CNR-UNIBS/mirrt_star) is found at build time (optional),
`graph::mirrt_star::MultigoalSolver` (MI-RRT\*, here with a single goal and its defaults: bidirectional, mixed
strategy with a tube around the solution, `rewire_radius` = 2·`max_distance`) on the same problems, with the same `InformedSampler`, Euclidean metrics, collision checker (axis-aligned boxes, edges checked
every 0.005) and parameters (`max_distance` 1.0, `rewire_factor` 1.1, `samples_per_batch` 100, r-disc RGG).
The problems (`<n>` is any dimension ≥ 2):

- `wall2d`: wall |x| ≤ 0.05 with a slit |y| < 0.1, from (-1, 0.8) to (1, 0.8) in [-1.5, 1.5]²;
- `cube<n>d`: obstacle [-1, 1]ⁿ, from (-1.5, …) to (1.5, …) in [-2.5, 2.5]ⁿ; the optimum bends once on an
  (n-2)-face of the cube, 2·√(0.5² + 2.5² + (n-2)·1.5²);
- `twocubes<n>d`: two boxes x₁ ∈ [0.05, 2.05] and x₁ ∈ [-2.05, -0.05] (the other coordinates in [-1, 1]) with a
  narrow passage |x₁| < 0.05 between them, from (-1.5, 0.6, 0, …) to (1.5, 0.6, 0, …) in [-2.5, 2.5]ⁿ; the
  optimum goes through the passage, 2·hypot(0.5, 0.55) + 2 ≈ 3.4866 in any dimension (around the boxes: 4.236).

The optima were also checked numerically (shortest path with free waypoints, up to 18D).

Results over 50 s. Each cell is the median cost over 7 runs, with the worst run in brackets (`inf` = no solution
yet). Below it are the median number of iterations (calls to `update()`) and, for BIT\*, of batches. An iteration
is not the same unit for all solvers: for BIT\* it is one edge (or one vertex expansion, or the drawing of a
batch), for RRT\* and MI-RRT\* one sample. The rows "md 10" use `max_distance` 10 (5 runs): with 1.0, RRT\*
finds no solution in 12D and 18D, and MI-RRT\* improves little. Release build, desktop CPU, 11 runs in parallel
(one per core). All solvers are informed: after the first solution they sample in the informed ellipsoid.

| Problem (optimum) | Solver | first sol. | 0.1 s | 1 s | 5 s | 10 s | 20 s | 50 s |
|---|---|---|---|---|---|---|---|---|
| wall2d (2.4601) | BIT\* | 0.30 ms | 2.477 [2.490]<br>39k it, 34 b | 2.467 [2.469]<br>145k it, 102 b | 2.466 [2.468]<br>360k it, 222 b | 2.464 [2.466]<br>563k it, 327 b | 2.463 [2.465]<br>835k it, 462 b | 2.463 [2.464]<br>1.4M it, 714 b |
|  | RRT\* | 0.28 ms | 2.482 [2.515]<br>4k it | 2.472 [2.482]<br>18k it | 2.469 [2.470]<br>56k it | 2.467 [2.469]<br>93k it | 2.464 [2.466]<br>152k it | 2.463 [2.464]<br>293k it |
|  | MI-RRT\* | 0.06 ms | 2.468 [2.475]<br>735 it | 2.465 [2.466]<br>2k it | 2.462 [2.463]<br>4k it | 2.461 [2.462]<br>5k it | 2.461 [2.462]<br>6k it | 2.460 [2.461]<br>9k it |
| cube3d (5.9161) | BIT\* | 0.35 ms | 6.015 [6.050]<br>18k it, 27 b | 6.002 [6.014]<br>77k it, 72 b | 5.969 [5.979]<br>237k it, 156 b | 5.965 [5.976]<br>383k it, 219 b | 5.961 [5.968]<br>619k it, 307 b | 5.960 [5.965]<br>1.1M it, 472 b |
|  | RRT\* | 0.45 ms | 6.167 [6.262]<br>4k it | 6.096 [6.148]<br>18k it | 6.047 [6.089]<br>53k it | 6.041 [6.065]<br>88k it | 6.031 [6.052]<br>150k it | 6.021 [6.029]<br>297k it |
|  | MI-RRT\* | 0.04 ms | 5.948 [5.969]<br>2k it | 5.936 [5.968]<br>6k it | 5.934 [5.946]<br>10k it | 5.928 [5.946]<br>14k it | 5.927 [5.935]<br>17k it | 5.925 [5.935]<br>24k it |
| cube6d (7.8740) | BIT\* | 0.24 ms | 8.360 [8.442]<br>965 it, 14 b | 8.302 [8.442]<br>3k it, 32 b | 8.294 [8.387]<br>5k it, 55 b | 8.289 [8.387]<br>7k it, 71 b | 8.283 [8.387]<br>9k it, 91 b | 8.283 [8.387]<br>14k it, 122 b |
|  | RRT\* | 51.4 ms | 9.632 [inf]<br>2k it | 9.370 [9.697]<br>9k it | 9.368 [9.396]<br>19k it | 9.358 [9.370]<br>28k it | 9.252 [9.370]<br>42k it | 9.155 [9.370]<br>78k it |
|  | MI-RRT\* | 0.04 ms | 8.499 [9.290]<br>1k it | 8.167 [8.272]<br>4k it | 8.075 [8.150]<br>8k it | 8.074 [8.108]<br>10k it | 8.064 [8.095]<br>13k it | 8.058 [8.077]<br>20k it |
| cube12d (10.7703) | BIT\* | 0.21 ms | 11.941 [12.268]<br>314 it, 11 b | 11.856 [11.941]<br>667 it, 26 b | 11.763 [11.898]<br>1k it, 46 b | 11.740 [11.898]<br>1k it, 55 b | 11.740 [11.898]<br>1k it, 66 b | 11.740 [11.898]<br>2k it, 84 b |
|  | RRT\* | – | inf [inf]<br>1k it | inf [inf]<br>4k it | inf [inf]<br>8k it | inf [inf]<br>10k it | inf [inf]<br>13k it | inf [inf]<br>18k it |
|  | RRT\* md 10 | 0.42 ms | 13.989 [14.714]<br>1k it | 13.989 [14.714]<br>5k it | 13.989 [14.714]<br>10k it | 13.989 [14.714]<br>12k it | 13.989 [14.714]<br>15k it | 13.989 [14.714]<br>20k it |
|  | MI-RRT\* | 0.07 ms | 14.253 [15.345]<br>1k it | 14.253 [15.345]<br>5k it | 14.100 [15.345]<br>9k it | 14.100 [15.345]<br>12k it | 14.100 [15.345]<br>16k it | 14.100 [15.345]<br>25k it |
|  | MI-RRT\* md 10 | 0.04 ms | 11.053 [11.101]<br>824 it | 11.008 [11.086]<br>3k it | 11.008 [11.026]<br>6k it | 10.984 [11.011]<br>7k it | 10.971 [10.984]<br>10k it | 10.958 [10.984]<br>14k it |
| cube18d (13.0384) | BIT\* | 0.24 ms | 14.371 [14.667]<br>246 it, 10 b | 14.371 [14.667]<br>514 it, 19 b | 14.371 [14.667]<br>803 it, 33 b | 14.371 [14.667]<br>939 it, 40 b | 14.371 [14.667]<br>1k it, 49 b | 14.371 [14.667]<br>1k it, 62 b |
|  | RRT\* | – | inf [inf]<br>663 it | inf [inf]<br>3k it | inf [inf]<br>6k it | inf [inf]<br>8k it | inf [inf]<br>10k it | inf [inf]<br>15k it |
|  | RRT\* md 10 | 0.86 ms | 17.867 [18.896]<br>1k it | 17.867 [18.443]<br>4k it | 17.867 [18.443]<br>9k it | 17.867 [18.443]<br>10k it | 17.867 [18.443]<br>12k it | 17.867 [18.443]<br>16k it |
|  | MI-RRT\* | 0.09 ms | 17.426 [20.002]<br>860 it | 17.338 [20.002]<br>4k it | 17.338 [20.002]<br>8k it | 17.338 [20.002]<br>11k it | 17.338 [20.002]<br>13k it | 17.338 [20.002]<br>18k it |
|  | MI-RRT\* md 10 | 0.05 ms | 13.512 [13.551]<br>875 it | 13.416 [13.500]<br>4k it | 13.388 [13.468]<br>9k it | 13.388 [13.468]<br>12k it | 13.388 [13.430]<br>17k it | 13.352 [13.371]<br>23k it |
| twocubes2d (3.4866) | BIT\* | 0.51 ms | 3.503 [3.506]<br>48k it, 28 b | 3.495 [3.498]<br>224k it, 84 b | 3.489 [3.493]<br>633k it, 182 b | 3.489 [3.492]<br>984k it, 256 b | 3.489 [3.492]<br>1.5M it, 365 b | 3.487 [3.489]<br>2.7M it, 577 b |
|  | RRT\* | 0.50 ms | 3.509 [5.173]<br>7k it | 3.494 [3.499]<br>28k it | 3.490 [3.492]<br>75k it | 3.489 [3.490]<br>120k it | 3.488 [3.490]<br>191k it | 3.486 [3.488]<br>360k it |
|  | MI-RRT\* | 0.08 ms | 5.122 [7.462]<br>2k it | 5.089 [7.415]<br>5k it | 5.077 [7.415]<br>10k it | 5.077 [7.415]<br>12k it | 5.076 [7.404]<br>17k it | 5.076 [7.402]<br>26k it |
| twocubes3d (3.4866) | BIT\* | 0.44 ms | 3.557 [3.574]<br>22k it, 27 b | 3.533 [3.544]<br>103k it, 71 b | 3.521 [3.530]<br>320k it, 138 b | 3.517 [3.525]<br>529k it, 192 b | 3.513 [3.522]<br>898k it, 269 b | 3.510 [3.518]<br>1.7M it, 414 b |
|  | RRT\* | 0.47 ms | 3.856 [4.581]<br>7k it | 3.601 [3.694]<br>34k it | 3.566 [3.603]<br>88k it | 3.546 [3.565]<br>136k it | 3.539 [3.555]<br>212k it | 3.531 [3.546]<br>387k it |
|  | MI-RRT\* | 0.06 ms | 3.541 [4.315]<br>2k it | 3.506 [4.263]<br>8k it | 3.500 [3.519]<br>17k it | 3.500 [3.509]<br>22k it | 3.499 [3.504]<br>30k it | 3.497 [3.502]<br>44k it |
| twocubes6d (3.4866) | BIT\* | 0.70 ms | 5.299 [5.492]<br>2k it, 17 b | 4.930 [5.014]<br>5k it, 41 b | 4.279 [4.767]<br>13k it, 91 b | 3.932 [4.681]<br>20k it, 128 b | 3.914 [4.092]<br>27k it, 167 b | 3.839 [3.935]<br>39k it, 236 b |
|  | RRT\* | 185.7 ms | inf [inf]<br>3k it | 6.321 [inf]<br>12k it | 5.999 [6.603]<br>24k it | 5.999 [6.370]<br>33k it | 5.963 [6.250]<br>49k it | 5.933 [6.091]<br>84k it |
|  | MI-RRT\* | 0.08 ms | 4.587 [4.835]<br>3k it | 3.651 [3.760]<br>11k it | 3.608 [3.649]<br>24k it | 3.604 [3.648]<br>35k it | 3.581 [3.638]<br>49k it | 3.581 [3.638]<br>73k it |
| twocubes12d (3.4866) | BIT\* | 1.4 ms | 9.434 [10.289]<br>1k it, 11 b | 8.327 [8.474]<br>4k it, 29 b | 7.613 [8.187]<br>7k it, 55 b | 7.367 [7.529]<br>9k it, 69 b | 7.123 [7.373]<br>12k it, 89 b | 7.029 [7.327]<br>14k it, 111 b |
|  | RRT\* | – | inf [inf]<br>1k it | inf [inf]<br>4k it | inf [inf]<br>8k it | inf [inf]<br>10k it | inf [inf]<br>12k it | inf [inf]<br>18k it |
|  | RRT\* md 10 | 0.57 ms | 9.470 [9.557]<br>1k it | 8.329 [8.591]<br>6k it | 8.189 [8.443]<br>15k it | 8.189 [8.443]<br>20k it | 8.189 [8.443]<br>25k it | 8.189 [8.443]<br>35k it |
|  | MI-RRT\* | 0.19 ms | 9.929 [13.623]<br>986 it | 9.929 [13.623]<br>4k it | 9.929 [13.241]<br>9k it | 8.944 [13.241]<br>11k it | 8.944 [13.241]<br>15k it | 8.944 [13.241]<br>23k it |
|  | MI-RRT\* md 10 | 0.12 ms | 5.014 [5.383]<br>3k it | 3.989 [5.014]<br>15k it | 3.832 [3.867]<br>48k it | 3.820 [3.864]<br>80k it | 3.776 [3.820]<br>127k it | 3.736 [3.807]<br>251k it |
| twocubes18d (3.4866) | BIT\* | 2.2 ms | 14.475 [15.015]<br>1k it, 8 b | 14.008 [14.963]<br>2k it, 18 b | 13.629 [14.209]<br>4k it, 31 b | 13.372 [13.728]<br>6k it, 44 b | 12.288 [13.728]<br>8k it, 53 b | 12.288 [13.158]<br>12k it, 83 b |
|  | RRT\* | – | inf [inf]<br>755 it | inf [inf]<br>3k it | inf [inf]<br>6k it | inf [inf]<br>8k it | inf [inf]<br>10k it | inf [inf]<br>15k it |
|  | RRT\* md 10 | 1.1 ms | 14.060 [15.308]<br>1k it | 13.851 [14.500]<br>5k it | 12.821 [13.030]<br>13k it | 12.167 [13.030]<br>17k it | 12.028 [13.030]<br>21k it | 11.807 [12.325]<br>28k it |
|  | MI-RRT\* | 0.33 ms | 15.081 [22.463]<br>721 it | 15.081 [22.463]<br>4k it | 13.411 [22.463]<br>8k it | 13.411 [22.463]<br>9k it | 13.411 [22.463]<br>13k it | 13.411 [22.463]<br>17k it |
|  | MI-RRT\* md 10 | 0.16 ms | 6.009 [6.290]<br>2k it | 4.501 [5.463]<br>13k it | 4.371 [5.247]<br>42k it | 4.256 [5.210]<br>73k it | 4.184 [5.156]<br>117k it | 4.174 [5.156]<br>209k it |

**MI-RRT\*** is the best of the three on almost every problem, from 2D to 18D:
- it converges closest to the optimum: `wall2d` 2.460, `cube3d` 5.925, `cube6d` 8.058 at 50 s, against
  BIT\* 2.463, 5.960, 8.283;
- in 12D and 18D, with `max_distance` 10, it is the only one that goes through the passage (`twocubes12d`
  3.74, `twocubes18d` 4.17) and that gets close to the optimum of the cubes (10.96 and 13.35, against 11.74 and
  14.37 for BIT\*). It also runs 10–20 times more iterations than RRT\* md 10 (251k against 35k in
  `twocubes12d`);
- the exception is `twocubes2d`: the median run stays around the upper box (5.076, the best path in that
  homotopy class is 5.068), while BIT\* and RRT\* reach 3.487. The sampling in the tube around the current
  solution keeps it in the wrong homotopy class. In `twocubes3d` too the worst run is at 3.50, but the median
  is right.
- with `max_distance` 1.0 in 12D and 18D, MI-RRT\* finds a solution but improves it little (14.10, 17.34,
  8.94, 13.41): the steering distance matters more for it than for BIT\*, which has no `max_distance`.

**BIT\* against RRT\*:**
- **2D:** BIT\* is ahead in the first seconds, then the two converge to the same cost.
- **3D:** BIT\* is ahead at every instant: RRT\* needs 50 s to reach what BIT\* has at 1–5 s.
- **6D:** RRT\* stays far from the optimum (`cube6d` 9.16, `twocubes6d` 5.93, around the boxes). BIT\* goes
  through the passage of `twocubes6d` within 10 s (3.84 at 50 s).
- **12D and 18D:** BIT\* is better than RRT\* md 10 on the cubes and on `twocubes12d`, worse on `twocubes18d`.
- **Why BIT\* stalls in high dimension:** BIT\* makes very few iterations, about 14k in 50 s in 6D and 1–2k in
  12D/18D, against 1–3M in 2D/3D. The r-disc radius grows with the dimension: 3.8 in 12D and 5.8–10.7 in 18D,
  in a box of side 5. The graph is then almost complete, and each vertex expansion queues edges towards almost
  all the states. On `cube18d` the cost does not change in 50 s (62 batches). Lowering the cost of a batch in
  high dimension is the main open point (see `CHANGELOG.md`, Planned).

The test `BeatsRRTStarOnCubeObstacle` checks the `cube3d` result (median of 5 runs at 0.25 s). The samplers are
not seeded, so the numbers change slightly at every run. Reproduce one cell with e.g.
`bit_star_compare_rrt_star --runs 7 --solver bit twocubes6d` (50 s per run; `--solver` takes a comma-separated
list of `bit`, `rrt`, `mirrt`). With no problem names, all ten problems and all solvers run in sequence, about
3 hours.

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
│   └── compare_rrt_star.cpp                    anytime comparison with RRT* and MI-RRT*
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
