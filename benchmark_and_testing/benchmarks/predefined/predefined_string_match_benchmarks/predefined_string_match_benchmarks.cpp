// Unit test utils
#include <predefined_utils/benchmark/fixtures/string_match_bench_fixtures.h>

// Day 3 string match library
#include <lib3611/w1d3_string_match/naive_search.h>

// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <memory>
#include <algorithm>

// Qualify predefined fixtures
using namespace dte3611::predef::benchmarking::string_match::fixtures;

namespace alg = dte3611::string_match::algorithms;

// Define benchmark fixtures for sorting of a sorted collection
BENCHMARK_DEFINE_F(HelloWorldF, stlSearch)
(benchmark::State& st)
{
  for (auto const& _ : st)
    std::search(m_string.begin(), m_string.end(), m_sequence.begin(),
                m_sequence.end());
}

BENCHMARK_DEFINE_F(HelloWorldF, naiveSearch)
(benchmark::State& st)
{
  for (auto const & _ : st)
    alg::naive_search(m_string.begin(), m_string.end(), m_sequence.begin(),
                      m_sequence.end());
}


// Register Benchmark
BENCHMARK_REGISTER_F(HelloWorldF, stlSearch);

BENCHMARK_REGISTER_F(HelloWorldF, naiveSearch);

BENCHMARK_MAIN();
