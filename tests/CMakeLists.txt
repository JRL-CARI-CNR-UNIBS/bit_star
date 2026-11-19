find_package(GTest REQUIRED)
include(GoogleTest)

# BIT* solver and plugin
add_executable(test_bit_star src/test_bit_star.cpp)
target_link_libraries(test_bit_star PRIVATE ${PROJECT_NAME} GTest::gtest_main)

# The plugin test loads the library by name, as an application would. cnr_class_loader looks for it only in the
# LD_LIBRARY_PATH folders (and /usr/lib, /usr/local/lib), so the build folder is prepended.
target_compile_definitions(test_bit_star PRIVATE BIT_STAR_LIBRARY_NAME="$<TARGET_FILE_NAME:${PROJECT_NAME}>")
gtest_discover_tests(test_bit_star
    DISCOVERY_TIMEOUT 30
    PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:${PROJECT_NAME}>:$ENV{LD_LIBRARY_PATH}"
)
