// Unit test utils
#include <predefined_utils/testing/fixtures/graph_testing_fixtures.h>

// Day 5 graph path finding library
#include <lib3611/w1d5_graph_path_finding/dijkstra_shortest_paths.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>

// Qualify predefined fixtures
using namespace dte3611::predef::testing::graph::fixtures;

namespace alg = dte3611::graph::algorithms;


// DAGOneF
TEST_F(DAGOneF, dijkstraShortestPaths_FromAToE)
{
  auto paths = alg::dijkstraShortestPaths(gold->graph(), gold->a(), gold->e());
  auto gold_paths = gold->shortestPathsAEGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}

TEST_F(DAGOneF, dijkstraShortestPaths_FromAToC)
{
  auto paths = alg::dijkstraShortestPaths(gold->graph(), gold->a(), gold->c());
  auto gold_paths = gold->shortestPathsACGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}

// DAGTwoF
TEST_F(DAGTwoF, dijkstraShortestPaths_FromAToE)
{
  auto paths = alg::dijkstraShortestPaths(gold->graph(), gold->a(), gold->e());
  auto gold_paths = gold->shortestPathsAEGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}

TEST_F(DAGTwoF, dijkstraShortestPaths_FromAToC)
{
  auto paths = alg::dijkstraShortestPaths(gold->graph(), gold->a(), gold->c());
  auto gold_paths = gold->shortestPathsACGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}


// DAGThreeF
TEST_F(DAGThreeF, dijkstraShortestPaths_FromAToE)
{
  auto paths = alg::dijkstraShortestPaths(gold->graph(), gold->a(), gold->e());
  auto gold_paths = gold->shortestPathsAEGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}

TEST_F(DAGThreeF, dijkstraShortestPaths_FromAToC)
{
  auto paths = alg::dijkstraShortestPaths(gold->graph(), gold->a(), gold->c());
  auto gold_paths = gold->shortestPathsACGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}
