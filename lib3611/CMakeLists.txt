project(lib3611)

# Add library target
add_library(${PROJECT_NAME} INTERFACE )

# Headers
target_include_directories( ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> )

# Make ${PROJECT_NAME} available as a direct linkable target
add_library(dte3611::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

## Unit tests
#option(BUILD_WEEK1_DAY1_2_UNITTESTS "Build unittests Week 1 : Day 1 & 2" OFF)
#if(BUILD_WEEK1_DAY1_2_UNITTESTS)
#  add_subdirectory(testing)
#endif(BUILD_WEEK1_DAY1_2_UNITTESTS)

## Benchmarking
#option(BUILD_WEEK1_DAY1_2_BENCHMARKS "Build benchmarks Week 1 : Day 1 & 2" OFF)
#if(BUILD_WEEK1_DAY1_2_BENCHMARKS)
#  add_subdirectory(benchmarking)
#endif(BUILD_WEEK1_DAY1_2_BENCHMARKS)
