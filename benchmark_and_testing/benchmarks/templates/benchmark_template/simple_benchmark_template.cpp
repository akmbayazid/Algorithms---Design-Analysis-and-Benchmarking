// Benchmark utils
//#include <predef_utils/ ... .h>

// Library to test headers
// #include <lib3611/ ... .h>

// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <vector>
#include <memory>
#include <ranges>
#include <algorithm>
#include <iterator>


// Example function to benchmark
std::vector<int> myCopyFunction( std::vector<int> const& in_vector ) {
  std::vector<int> out_vector;
  out_vector.reserve(in_vector.size());
  std::ranges::copy(in_vector, std::back_inserter(out_vector));
  return out_vector;
}

// BM data
std::unique_ptr<std::vector<int>> in_vec;

// BM setup
static void bmDoSetup(const benchmark::State& state) {
  in_vec = std::make_unique<std::vector<int>>(state.range(0), state.range(0));
}

// BM teardown
static void bmDoTeardown(const benchmark::State& /*state*/) {
  in_vec.release();
}

// BM testfunction
static void bmFunc(benchmark::State& state) {

  for (auto _ : state) {
    std::vector<int> out = myCopyFunction(*in_vec);
    benchmark::DoNotOptimize(in_vec->data());
    benchmark::DoNotOptimize(out.data());
    benchmark::ClobberMemory();
  }
}

// Register tests
BENCHMARK(bmFunc)->DenseRange(0, 1024, 128)->Setup(bmDoSetup)->Teardown(bmDoTeardown);


// Benchmark main function
BENCHMARK_MAIN();
