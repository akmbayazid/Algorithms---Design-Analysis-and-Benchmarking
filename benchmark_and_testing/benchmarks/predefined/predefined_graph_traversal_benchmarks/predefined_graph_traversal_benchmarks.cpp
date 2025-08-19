// Unit test utils
#include <predefined_utils/benchmark/fixtures/graph_bench_fixtures.h>

// Day 4 graph traversal library
#include <lib3611/w1d4_graph_traversal/depth_first_search.h>
#include <lib3611/w1d4_graph_traversal/breadth_first_search.h>

// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <memory>
#include <algorithm>

// Qualify predefined fixtures
using namespace dte3611::predef::benchmarking::graph::fixtures;

namespace alg = dte3611::graph::algorithms;

// Define benchmark fixtures for sorting of a sorted collection
BENCHMARK_DEFINE_F(BenchDagOneF, breadthFirstSearch)
(benchmark::State& st)
{
  for (auto const& _ : st)
    [[maybe_unused]] auto const tree
      = alg::breadthFirstSearch(m_graph->graph(), m_graph->a());
}

BENCHMARK_DEFINE_F(BenchDagOneF, depthFirstSearch)
(benchmark::State& st)
{
  for (auto const& _ : st)
    [[maybe_unused]] auto const tree
      = alg::depthFirstSearch(m_graph->graph(), m_graph->a());
}


// Register Benchmark
BENCHMARK_REGISTER_F(BenchDagOneF, breadthFirstSearch);

BENCHMARK_REGISTER_F(BenchDagOneF, depthFirstSearch);

BENCHMARK_MAIN();
