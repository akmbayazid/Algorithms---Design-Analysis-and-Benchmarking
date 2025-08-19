// Unit test utils
#include <predefined_utils/testing/fixtures/graph_testing_fixtures.h>

// Day 4 graph traversal library
#include <lib3611/w1d4_graph_traversal/depth_first_search.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>


// Qualify predefined fixtures
using namespace dte3611::predef::testing::graph::fixtures;

namespace alg =  dte3611::graph::algorithms;

TEST_F(SingleNodeDAGF, depthFirstSearch)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->gold());
}

TEST_F(TreeDAGF, depthFirstSearchFromRoot)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->dfsFromAGold());
}

// DAGOneF
TEST_F(DAGOneF, depthFirstSearch_FromA)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->dfsFromAGold());
}

TEST_F(DAGOneF, depthFirstSearch_FromB)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->b());
  EXPECT_EQ(tree, gold->dfsFromBGold());
}

TEST_F(DAGOneF, depthFirstSearch_FromC)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->c());
  EXPECT_EQ(tree, gold->dfsFromCGold());
}

TEST_F(DAGOneF, depthFirstSearch_FromD)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->d());
  EXPECT_EQ(tree, gold->dfsFromDGold());
}

TEST_F(DAGOneF, depthFirstSearch_FromE)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->e());
  EXPECT_EQ(tree, gold->dfsFromEGold());
}

TEST_F(DAGOneF, depthFirstSearch_FromF)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->f());
  EXPECT_EQ(tree, gold->dfsFromFGold());
}


// DAGTwoF
TEST_F(DAGTwoF, depthFirstSearch_FromA)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->dfsFromAGold());
}

TEST_F(DAGTwoF, depthFirstSearch_FromB)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->b());
  EXPECT_EQ(tree, gold->dfsFromBGold());
}

TEST_F(DAGTwoF, depthFirstSearch_FromC)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->c());
  EXPECT_EQ(tree, gold->dfsFromCGold());
}

TEST_F(DAGTwoF, depthFirstSearch_FromD)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->d());
  EXPECT_EQ(tree, gold->dfsFromDGold());
}

TEST_F(DAGTwoF, depthFirstSearch_FromE)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->e());
  EXPECT_EQ(tree, gold->dfsFromEGold());
}

TEST_F(DAGTwoF, depthFirstSearch_FromF)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->f());
  EXPECT_EQ(tree, gold->dfsFromFGold());
}

TEST_F(DAGTwoF, depthFirstSearch_FromG)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->g());
  EXPECT_EQ(tree, gold->dfsFromGGold());
}

TEST_F(DAGTwoF, depthFirstSearch_FromH)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->h());
  EXPECT_EQ(tree, gold->dfsFromHGold());
}


// DAGThreeF
TEST_F(DAGThreeF, depthFirstSearch_FromA)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->dfsFromAGold());
}

TEST_F(DAGThreeF, depthFirstSearch_FromB)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->b());
  EXPECT_EQ(tree, gold->dfsFromBGold());
}

TEST_F(DAGThreeF, depthFirstSearch_FromC)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->c());
  EXPECT_EQ(tree, gold->dfsFromCGold());
}

TEST_F(DAGThreeF, depthFirstSearch_FromD)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->d());
  EXPECT_EQ(tree, gold->dfsFromDGold());
}

TEST_F(DAGThreeF, depthFirstSearch_FromE)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->e());
  EXPECT_EQ(tree, gold->dfsFromEGold());
}

TEST_F(DAGThreeF, depthFirstSearch_FromF)
{
  auto const tree = alg::depthFirstSearch(gold->graph(), gold->f());
  EXPECT_EQ(tree, gold->dfsFromFGold());
}
