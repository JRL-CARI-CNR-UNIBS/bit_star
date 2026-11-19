# graph_core brings Eigen3, yaml-cpp and cnr_class_loader as transitive dependencies of graph_core::graph_core.
# Dependencies are not fetched at configure time (no CPM / FetchContent): install them first, in the colcon
# workspace (deps.repos) or in a prefix listed in CMAKE_PREFIX_PATH. bit_star needs the graph_core branch with
# ParamMap and the abstract logger (see deps.repos).
find_package(graph_core REQUIRED)
