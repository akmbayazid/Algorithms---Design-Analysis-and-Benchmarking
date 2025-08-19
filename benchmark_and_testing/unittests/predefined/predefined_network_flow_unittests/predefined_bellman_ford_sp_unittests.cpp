// Unit test utils
#include <predefined_utils/testing/fixtures/graph_testing_fixtures.h>

// W3 graph path finding library
#include <lib3611/w3_network_flow/bellman_ford.h>

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
  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->a(), gold->e());
  auto gold_paths = gold->shortestPathsAEGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}

TEST_F(DAGOneF, dijkstraShortestPaths_FromAToC)
{
  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->a(), gold->c());
  auto gold_paths = gold->shortestPathsACGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}

// DAGTwoF
TEST_F(DAGTwoF, dijkstraShortestPaths_FromAToE)
{
  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->a(), gold->e());
  auto gold_paths = gold->shortestPathsAEGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}

TEST_F(DAGTwoF, dijkstraShortestPaths_FromAToC)
{
  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->a(), gold->c());
  auto gold_paths = gold->shortestPathsACGold();
  std::sort(std::begin(paths), std::end(paths));
  std::sort(std::begin(gold_paths), std::end(gold_paths));
  EXPECT_EQ(paths, gold_paths);
}


// DAGThreeF
//TEST_F(DAGThreeF, dijkstraShortestPaths_FromAToE)
//{
//  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->a(), gold->e());
//  auto gold_paths = gold->shortestPathsAEGold();
//  std::sort(std::begin(paths), std::end(paths));
//  std::sort(std::begin(gold_paths), std::end(gold_paths));
//  EXPECT_EQ(paths, gold_paths);
//}

//TEST_F(DAGThreeF, dijkstraShortestPaths_FromAToC)
//{
//  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->a(), gold->c());
//  auto gold_paths = gold->shortestPathsACGold();
//  std::sort(std::begin(paths), std::end(paths));
//  std::sort(std::begin(gold_paths), std::end(gold_paths));
//  EXPECT_EQ(paths, gold_paths);
//}

// Dag_Neg_One
TEST_F(DAG_Neg_OneF, bellmanFordShortestPaths_FromSToT)
{
  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->s(), gold->t());
  auto gold_paths = gold->shortestPathsSTGold();
  EXPECT_EQ(paths, gold_paths);

}

// Dag_Neg_Two
TEST_F(DAG_Neg_TwoF, bellmanFordShortestPaths_FromSToT)
{
  auto paths = alg::bellmanFordShortestPaths(gold->graph(), gold->s(), gold->t());
  auto gold_paths = gold->shortestPathsSTGold();
  EXPECT_EQ(paths, gold_paths);

}
