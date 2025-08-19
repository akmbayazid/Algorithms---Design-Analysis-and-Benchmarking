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

// Template fixture struct
struct MyTemplateBenchmarkF : benchmark::Fixture {

  using benchmark::Fixture::Fixture;
  ~MyTemplateBenchmarkF() override {}

  std::unique_ptr<std::vector<int>> m_data;

  void SetUp(const benchmark::State& state) final
  {
    m_data = std::make_unique<std::vector<int>>(state.range(0), state.range(0));
  }

  void TearDown(const benchmark::State&) final { m_data.release(); }
};


// Define benchmark fixtures for sorting of a sorted collection
BENCHMARK_DEFINE_F(MyTemplateBenchmarkF, copyFuncBenchSetupOne)
(benchmark::State& state)
{
  for (auto _ : state) {
    std::vector<int> out = myCopyFunction(*m_data);
    benchmark::DoNotOptimize(m_data->data());
    benchmark::DoNotOptimize(out.data());
    benchmark::ClobberMemory();
  }
}


// Register Benchmark : benchmark sorting of a sorted collection using different
// algorithms
BENCHMARK_REGISTER_F(MyTemplateBenchmarkF, copyFuncBenchSetupOne)
  ->DenseRange(0, 1024, 128);

BENCHMARK_MAIN();
