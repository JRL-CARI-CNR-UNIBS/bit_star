# Changelog
All notable changes to `bit_star` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Planned
- Options of the OMPL implementation not available yet: delay the rewiring until the first solution, just-in-time
  sampling, drop the unconnected samples at each pruning, stop at each solution improvement, approximate
  solutions.
- Cheaper batches in high dimension (12D and more): the r-disc radius makes the graph almost complete, and BIT\*
  runs only 1–2k iterations in 50 s (see the comparison in the README).
- Shared base for the planners derived from BIT\* (ABIT\*, AIT\*, EIT\*): batches, RGG radius, per-vertex labels.

## [0.1.0] - 2026-11-19

### Added
- `graph::bit_star::BITStar`, BIT\* as a `graph::core::TreeSolver`: batches of collision-free samples in the
  informed set, r-disc or k-nearest RGG, vertex and edge queues ordered by the estimated solution cost, lazy
  collision checking, pruning of the samples and vertices that cannot improve the solution, anytime `solve()`.
- `direct_connection` parameter, on by default: the edge start-goal is checked when the problem is set, as
  `TreeSolver::setProblem()` does. Without it the goal is reached only through the RGG, as in the original
  algorithm.
- Cached cost-to-come per vertex, propagated to the descendants on rewiring, instead of `Tree::costToNode()` at
  every queue operation.
- Lazy re-keying of the queues with a version number per vertex.
- Cache of the edges found in collision, never checked twice.
- Warm start from an existing tree (`addStartTree()`), a path (`setSolution()`) or another solver
  (`importFromSolver()`).
- `graph::bit_star::BITStarPlugin`, loadable with `cnr_class_loader` as a `graph::core::TreeSolverPlugin`.
- Example `bit_star_wall_example` (wall with a slit in the plane), gtest suite, tutorial, API reference, CI.
- Example `bit_star_compare_rrt_star` and test `BeatsRRTStarOnCubeObstacle`: anytime comparison with graph_core's
  RRT\* and, if mirrt_star is installed, with MI-RRT\*, on a wall with a slit (2D), a cube obstacle and two boxes
  with a narrow passage, in any dimension (2D to 18D in the README);
  results in the README.
