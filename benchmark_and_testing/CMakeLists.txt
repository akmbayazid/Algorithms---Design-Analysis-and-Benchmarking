
add_subdirectory(predefined_utils)

include(../cmake/testing_utils.cmake)

option(DTE3611_ENABLE_UNITTESTS "Enable : Unittests" OFF)
if(DTE3611_ENABLE_UNITTESTS)

  ## Setup unittest environment variables used by the individual unittests
  SETUP_UNITTEST_ENV()

  ## Guard for GTEST
  if(GTest_FOUND)

    # Unittest template
    add_subdirectory(unittests/templates/unittest_template)

    # Predefined unittests
    add_subdirectory(unittests/predefined/predefined_sort_unittests)
    add_subdirectory(unittests/predefined/predefined_string_match_unittests)
    add_subdirectory(unittests/predefined/predefined_graph_traversal_unittests)
    add_subdirectory(unittests/predefined/predefined_graph_path_finding_unittests)
    add_subdirectory(unittests/predefined/predefined_subset_sum_unittests)
    add_subdirectory(unittests/predefined/predefined_knapsack_unittests)
    add_subdirectory(unittests/predefined/predefined_network_flow_unittests)

    # My Unittests
    add_subdirectory(unittests/my_tests/my_sort_tests)

  endif(GTest_FOUND)
endif(DTE3611_ENABLE_UNITTESTS)

## Guard for BENCHMARK
option(DTE3611_ENABLE_BENCHMARKS "Enable : Benchmarks" OFF)
if(DTE3611_ENABLE_BENCHMARKS)
  SETUP_BENCHMARK_ENV()

  SET(MSVC_COMPILE_OPTIONS
    # REQUIRED ON
    /W4 /permissive-
    # REQUIRED ON
  )

  ## Guard for Google benchmark
  if(benchmark_FOUND)

    # Benchmark template
    add_subdirectory(benchmarks/templates/benchmark_template)

    # Predefined benchmarks
    add_subdirectory(benchmarks/predefined/predefined_sort_benchmarks)
    add_subdirectory(benchmarks/predefined/predefined_string_match_benchmarks)
    add_subdirectory(benchmarks/predefined/predefined_graph_traversal_benchmarks)
    add_subdirectory(benchmarks/predefined/predefined_graph_path_finding_benchmarks)

    # My Unittests
    add_subdirectory(benchmarks/my_benchmarks/my_first_benchmarks)

  endif(benchmark_FOUND)

endif(DTE3611_ENABLE_BENCHMARKS)
