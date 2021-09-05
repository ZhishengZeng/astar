cmake_minimum_required(VERSION 3.11)
project(ASTAR)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/third_party/googletest
)

add_library(astar
${PROJECT_SOURCE_DIR}/src/Model.cpp
${PROJECT_SOURCE_DIR}/src/Util.cpp
)


# SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fsanitize=address")

add_executable(simple_astar ${PROJECT_SOURCE_DIR}/tests/simple.cpp)
add_executable(random_astar ${PROJECT_SOURCE_DIR}/tests/random.cpp)
target_link_libraries(simple_astar astar)
target_link_libraries(random_astar astar)

add_executable(test_astar ${PROJECT_SOURCE_DIR}/tests/gtest.cpp)
link_directories(${PROJECT_SOURCE_DIR}/third_party/googletest)
target_link_libraries(test_astar libgtest.a libgtest_main.a pthread astar)



