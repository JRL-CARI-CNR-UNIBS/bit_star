cmake_minimum_required(VERSION 3.16)
project(bit_star VERSION 0.1.0 LANGUAGES CXX)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Compiler flags (same as graph_core)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(CMAKE_BUILD_TYPE MATCHES "Release")
        add_compile_options(-funroll-loops -Wall -Wextra -flto -O3)
    endif()
    if(CMAKE_BUILD_TYPE MATCHES "Debug")
        add_compile_options(-Wall -Wextra -g -Og)
    endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(BUILD_TESTS "Build the gtest suite" ON)
option(BUILD_EXAMPLES "Build the example executables" ON)

# Dependencies
include(cmake_config/Dependencies.cmake)

# Configure RPATH for libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Library target: the solver and its cnr_class_loader plugin
add_library(${PROJECT_NAME} SHARED
    # Solvers
    src/${PROJECT_NAME}/solvers/bit_star.cpp

    # Plugins
    src/${PROJECT_NAME}/plugins/solvers/bit_star_plugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    graph_core::graph_core
)

# Alias for library
add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

# Examples
if(BUILD_EXAMPLES)
    # Wall with a narrow slit in the plane, see docs/TUTORIAL.md
    add_executable(bit_star_wall_example examples/wall_example.cpp)
    target_link_libraries(bit_star_wall_example PRIVATE ${PROJECT_NAME})
    install(TARGETS bit_star_wall_example RUNTIME DESTINATION lib/${PROJECT_NAME})

    # Anytime comparison with graph_core's RRT* and, if installed, with MI-RRT* (results in the README)
    add_executable(bit_star_compare_rrt_star examples/compare_rrt_star.cpp)
    target_link_libraries(bit_star_compare_rrt_star PRIVATE ${PROJECT_NAME})
    find_package(mirrt_star QUIET)
    if(mirrt_star_FOUND)
        message(STATUS "mirrt_star found: bit_star_compare_rrt_star compares also MI-RRT*")
        target_link_libraries(bit_star_compare_rrt_star PRIVATE mirrt_star::mirrt_star)
        target_compile_definitions(bit_star_compare_rrt_star PRIVATE BIT_STAR_WITH_MIRRT_STAR)
    else()
        message(STATUS "mirrt_star not found: bit_star_compare_rrt_star compares BIT* and RRT* only")
    endif()
    install(TARGETS bit_star_compare_rrt_star RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

# Tests (run with ctest, or colcon test)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install headers
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)

# Install targets
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install exported targets
install(EXPORT ${PROJECT_NAME}Targets
    DESTINATION "share/${PROJECT_NAME}/cmake"
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
)

# Package configuration
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "share/${PROJECT_NAME}/cmake"
)

# Clang-format target
add_custom_target(
    format
    COMMAND chmod +x cmake_config/clang_format_target.sh
    COMMAND cmake_config/clang_format_target.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running clang-format on all source files"
)
