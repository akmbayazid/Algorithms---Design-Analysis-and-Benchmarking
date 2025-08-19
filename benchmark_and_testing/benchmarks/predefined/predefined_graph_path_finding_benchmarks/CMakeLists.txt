####################################
# Automatic component project naming
get_filename_component(FNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${FNAME})


###################
# Benchmark setings
set( LIB_TO_BENCHMARK
  dte3611::lib3611 )

set( BENCHMARKS
  predefined_graph_path_finding_benchmarks )

set( OTHER_LINK_TARGETS
  dte3611::predefined_utils )


#######################
# Benchmark build driver
option(DTE3611_BUILD_BENCHMARK_${FNAME} "Build benchmark: ${FNAME}" OFF)
if(DTE3611_BUILD_BENCHMARK_${FNAME})
  ADD_BENCHMARKS( ${LIB_TO_BENCHMARK} BENCHMARKS ${OTHER_LINK_TARGETS} )
endif(DTE3611_BUILD_BENCHMARK_${FNAME})

