macro(SETUP_UNITTEST_ENV)

  # Enable testing
  enable_testing()

  # Configure threads
  find_package(Threads REQUIRED)

  # Configure GTest
  find_package(GTest REQUIRED)
  if(GTest_FOUND)

    # Turn off static analysis for benchmark tool (no compilation warnings)
    target_compile_options( GTest::GTest
        INTERFACE $<$<CXX_COMPILER_ID:Clang>: -w >
        INTERFACE $<$<CXX_COMPILER_ID:GNU>: -w >
  #        INTERFACE $<$<CXX_COMPILER_ID:MSVC>: /w > # Turn on if MSVC complains
      )

    target_compile_options( GTest::Main
        INTERFACE $<$<CXX_COMPILER_ID:Clang>: -w >
        INTERFACE $<$<CXX_COMPILER_ID:GNU>: -w >
  #        INTERFACE $<$<CXX_COMPILER_ID:MSVC>: /w > # Turn on if MSVC complains
      )

    # GTEST compile options
    set( GTEST_CLANG_COMPILE_OPTIONS
      # option
      )
    set( GTEST_GCC_COMPILE_OPTIONS
      # option
      )
    set( GTEST_MSVC_COMPILE_OPTIONS
      # option
      )

  endif(GTest_FOUND)
endmacro(SETUP_UNITTEST_ENV)


function(ADD_UNITTESTS UNITTEST_LIB_TOTEST UNITTESTS_INN)
  # Add tests
  # -- ARGN :: additional LIB dependencies

  foreach(TEST ${${UNITTESTS_INN}})

    set( TESTNAME unittest_${TEST} )
    add_executable( ${TESTNAME} )

    target_sources( ${TESTNAME}
      PRIVATE ${TEST}.cpp ${SRCS} )

    target_link_libraries( ${TESTNAME}
      PRIVATE
        ${UNITTEST_LIB_TOTEST}
        ${ARGN}
        GTest::GTest GTest::Main
        Threads::Threads )

    # C++ standard feature set
    target_compile_features( ${TESTNAME}
      PUBLIC $<$<CXX_COMPILER_ID:Clang>:cxx_std_20>
      PUBLIC $<$<CXX_COMPILER_ID:GNU>:cxx_std_20>
      PUBLIC $<$<CXX_COMPILER_ID:MSVC>:cxx_std_20>
      )

    # Compile definitions
    target_compile_definitions( ${TESTNAME}
      PUBLIC $<$<CXX_COMPILER_ID:MSVC>: _USE_MATH_DEFINES>
      )

    # Comple options
    target_compile_options( ${TESTNAME}
      PUBLIC $<$<CXX_COMPILER_ID:Clang>: ${CLANG_COMPILE_OPTIONS} ${GTEST_CLANG_COMPILE_OPTIONS}>
      PUBLIC $<$<CXX_COMPILER_ID:GNU>:   ${GCC_COMPILE_OPTIONS}   ${GTEST_GCC_COMPILE_OPTIONS}>
      PUBLIC $<$<CXX_COMPILER_ID:MSVC>:  ${MSVC_COMPILE_OPTIONS}  ${GTEST_MSVC_COMPILE_OPTIONS}>
      )

    # Turn off platform-spesific extensions
    set_target_properties(${TESTNAME} PROPERTIES CXX_EXTENSIONS OFF)

    gtest_discover_tests( ${TESTNAME} "" AUTO )

  endforeach(TEST)

endfunction(ADD_UNITTESTS)



macro(SETUP_BENCHMARK_ENV)

  # Enable testing
  enable_testing()

  # Configure threads
  find_package(Threads REQUIRED)

  # Configure benchmark
  find_package(benchmark REQUIRED)
  if(benchmark_FOUND)

    # Turn off static analysis for benchmark tool (no compilation warnings)
    target_compile_options( benchmark::benchmark
        INTERFACE $<$<CXX_COMPILER_ID:Clang>: -w >
        INTERFACE $<$<CXX_COMPILER_ID:GNU>: -w >
#        INTERFACE $<$<CXX_COMPILER_ID:MSVC>: /w > # Turn on if MSVC complains
      )

    # benchmark compile options
    set( BENCHMARK_CLANG_COMPILE_OPTIONS
      # option
      )
    set( BENCHMARK_GCC_COMPILE_OPTIONS
      # option
      )
    set( BENCHMARK_MSVC_COMPILE_OPTIONS
      # option
      )

  endif(benchmark_FOUND)

endmacro(SETUP_BENCHMARK_ENV)



function(ADD_BENCHMARKS BENCHMARKS_LIB_TOTEST BENCHMARKS_INN)
  # Add tests
  # -- ARGN :: additional LIB dependencies

  foreach(BENCHMARK ${${BENCHMARKS_INN}})
    set(BNAME  benchmark_${BENCHMARK})
    add_executable( ${BNAME} )

    target_sources( ${BNAME} PRIVATE ${BENCHMARK}.cpp )

    target_link_libraries( ${BNAME}
      PRIVATE
        ${BENCHMARKS_LIB_TOTEST}
        ${ARGN}
        benchmark::benchmark
        Threads::Threads )

    # Turn on c++20 compile features -- minimum CMake Version 3.8 required
    target_compile_features( ${BNAME}
      PUBLIC $<$<CXX_COMPILER_ID:Clang>:cxx_std_20>
      PUBLIC $<$<CXX_COMPILER_ID:GNU>:cxx_std_20>
      PUBLIC $<$<CXX_COMPILER_ID:MSVC>:cxx_std_20>
      )

    # Compile definitions
    target_compile_definitions( ${BNAME}
      PUBLIC $<$<CXX_COMPILER_ID:MSVC>: _USE_MATH_DEFINES>
      )

    # Comple options
    target_compile_options( ${BNAME}
      PUBLIC $<$<CXX_COMPILER_ID:Clang>: ${CLANG_COMPILE_OPTIONS} ${BENCHMARK_CLANG_COMPILE_OPTIONS}>
      PUBLIC $<$<CXX_COMPILER_ID:GNU>:   ${GCC_COMPILE_OPTIONS}   ${BENCHMARK_GCC_COMPILE_OPTIONS}>
      PUBLIC $<$<CXX_COMPILER_ID:MSVC>:  ${MSVC_COMPILE_OPTIONS}  ${BENCHMARK_MSVC_COMPILE_OPTIONS}>
      )

    # Turn off platform-spesific extensions
    set_target_properties(${BNAME} PROPERTIES CXX_EXTENSIONS OFF)

    add_test( NAME ${BNAME} COMMAND ${BNAME} )
  endforeach(BENCHMARK ${BENCHMARKS})

endfunction(ADD_BENCHMARKS)

