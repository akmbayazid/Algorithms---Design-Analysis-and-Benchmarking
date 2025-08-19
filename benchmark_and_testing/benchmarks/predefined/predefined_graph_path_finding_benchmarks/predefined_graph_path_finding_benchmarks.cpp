// Unit test utils
#include <predefined_utils/benchmark/fixtures/graph_bench_fixtures.h>

// Day 5 graph traversal library
#include <lib3611/w1d5_graph_path_finding/dijkstra_shortest_paths.h>
#include <lib3611/w1d5_graph_path_finding/astar_search.h>

// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <memory>
#include <algorithm>

// Qualify predefined fixtures
using namespace dte3611::predef::benchmarking::graph::fixtures;

namespace alg = dte3611::graph::algorithms;

// A heuristic predicate
const auto unit_heuristic_fn
  = []([[maybe_unused]] auto const& u, [[maybe_unused]] auto const& v,
       [[maybe_unused]] auto const& graph) { return 1.; };

// Define benchmark fixtures for sorting of a sorted collection
BENCHMARK_DEFINE_F(BenchDagOneF, dijkstraShortestPaths)
(benchmark::State& st)
{
  for (auto const& _ : st)
    [[maybe_unused]] auto paths = alg::dijkstraShortestPaths(
      m_graph->graph(), m_graph->a(), m_graph->e());
}

BENCHMARK_DEFINE_F(BenchDagOneF, aStarSearch)
(benchmark::State& st)
{
  for (auto const& _ : st)
    [[maybe_unused]] auto paths = alg::aStarSearch(
      m_graph->graph(), m_graph->a(), m_graph->e(), unit_heuristic_fn);
}


// Register Benchmark
BENCHMARK_REGISTER_F(BenchDagOneF, dijkstraShortestPaths);

BENCHMARK_REGISTER_F(BenchDagOneF, aStarSearch);

BENCHMARK_MAIN();
