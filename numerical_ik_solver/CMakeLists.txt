cmake_minimum_required(VERSION 2.8.0)

project(testrunner)


find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

#NumericalIkSolver
add_executable(testrunner testrunner.cpp NumericalIkSolver.cpp)
target_link_libraries(testrunner ${CERES_LIBRARIES})
