cmake_minimum_required(VERSION 3.8)
project(finder_bots)

include ( CMakePackageConfigHelpers )

# Check if compiler is GCC or Clang, then add extra warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile_commands.json for linting
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(GTest REQUIRED) # Add GTest
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catch_ros2 REQUIRED)  # Add Catch2
find_package(Doxygen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Doxygen Rules
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_CLASS_DIAGRAMS YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_CLASS_GRAPH YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_PACKAGE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
  set(DOXYGEN_TEMPLATE_RELATIONS YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
  set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
  set(DOXYGEN_DOT_TRANSPARENT YES)
else()
  message(FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation")
endif()

doxygen_add_docs(docs           # target name
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/test
)

if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

option(WANT_COVERAGE "this option enable coverage" ON)
if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  set(WANT_COVERAGE OFF)
endif()

if(WANT_COVERAGE)
  message("Enabling coverage")
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")
  include(CodeCoverage)

  append_coverage_compiler_flags()

  set(COVERAGE_EXCLUDES
    ${PROJECT_BINARY_DIR}
  )
endif()

# Sanity check:
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message(STATUS "WANT_COVERAGE    = ${WANT_COVERAGE}")

# Enables testing for this directory and below
enable_testing()
include(GoogleTest)

# Add executables
add_executable(finderBots src/finder_bots_master.cpp)
ament_target_dependencies(finderBots rclcpp cv_bridge sensor_msgs geometry_msgs OpenCV nav_msgs tf2 tf2_ros)

install(TARGETS
  finderBots
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Catch2 Tests
if(BUILD_TESTING)
  include(CTest)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)


  # GTest
  set(GTEST_FILES
    test/test.cpp
  )
  add_executable(gtest_finderBots ${GTEST_FILES})
  target_link_libraries(gtest_finderBots GTest::GTest GTest::Main)
  ament_target_dependencies(gtest_finderBots rclcpp std_msgs)
  gtest_discover_tests(gtest_finderBots)

  # Catch2
  set(TESTFILES
    test/test_lv2.cpp)

  add_executable(catch_test_finderBots ${TESTFILES})
  target_link_libraries(catch_test_finderBots catch_ros2::catch_ros2_with_node_main)
  ament_target_dependencies(catch_test_finderBots rclcpp std_msgs)

  install(TARGETS catch_test_finderBots
    DESTINATION lib/${PROJECT_NAME})

  catch_ros2_add_integration_test(${PROJECT_NAME}_test LAUNCH_FILE test.launch.py)
endif()

install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()
