// Unit test utils
#include <predefined_utils/testing/fixtures/graph_testing_fixtures.h>

// Day 4 graph traversal library
#include <lib3611/w1d4_graph_traversal/breadth_first_search.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>

// Qualify predefined fixtures
using namespace dte3611::predef::testing::graph::fixtures;

namespace alg = dte3611::graph::algorithms;

TEST_F(SingleNodeDAGF, breadthFirstSearch)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->gold());
}

TEST_F(TreeDAGF, breadthFirstSearchFromRoot)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->bfsFromAGold());
}


// DAGOneF
TEST_F(DAGOneF, breadthFirstSearch_FromA)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->bfsFromAGold());
}

TEST_F(DAGOneF, breadthFirstSearch_FromB)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->b());
  EXPECT_EQ(tree, gold->bfsFromBGold());
}

TEST_F(DAGOneF, breadthFirstSearch_FromC)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->c());
  EXPECT_EQ(tree, gold->bfsFromCGold());
}

TEST_F(DAGOneF, breadthFirstSearch_FromD)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->d());
  EXPECT_EQ(tree, gold->bfsFromDGold());
}

TEST_F(DAGOneF, breadthFirstSearch_FromE)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->e());
  EXPECT_EQ(tree, gold->bfsFromEGold());
}

TEST_F(DAGOneF, breadthFirstSearch_FromF)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->f());
  EXPECT_EQ(tree, gold->bfsFromFGold());
}


// DAGTwoF
TEST_F(DAGTwoF, breadthFirstSearch_FromA)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->bfsFromAGold());
}

TEST_F(DAGTwoF, breadthFirstSearch_FromB)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->b());
  EXPECT_EQ(tree, gold->bfsFromBGold());
}

TEST_F(DAGTwoF, breadthFirstSearch_FromC)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->c());
  EXPECT_EQ(tree, gold->bfsFromCGold());
}

TEST_F(DAGTwoF, breadthFirstSearch_FromD)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->d());
  EXPECT_EQ(tree, gold->bfsFromDGold());
}

TEST_F(DAGTwoF, breadthFirstSearch_FromE)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->e());
  EXPECT_EQ(tree, gold->bfsFromEGold());
}

TEST_F(DAGTwoF, breadthFirstSearch_FromF)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->f());
  EXPECT_EQ(tree, gold->bfsFromFGold());
}

TEST_F(DAGTwoF, breadthFirstSearch_FromG)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->g());
  EXPECT_EQ(tree, gold->bfsFromGGold());
}

TEST_F(DAGTwoF, breadthFirstSearch_FromH)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->h());
  EXPECT_EQ(tree, gold->bfsFromHGold());
}


// DAGThreeF
TEST_F(DAGThreeF, breadthFirstSearch_FromA)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->a());
  EXPECT_EQ(tree, gold->bfsFromAGold());
}

TEST_F(DAGThreeF, breadthFirstSearch_FromB)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->b());
  EXPECT_EQ(tree, gold->bfsFromBGold());
}

TEST_F(DAGThreeF, breadthFirstSearch_FromC)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->c());
  EXPECT_EQ(tree, gold->bfsFromCGold());
}

TEST_F(DAGThreeF, breadthFirstSearch_FromD)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->d());
  EXPECT_EQ(tree, gold->bfsFromDGold());
}

TEST_F(DAGThreeF, breadthFirstSearch_FromE)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->e());
  EXPECT_EQ(tree, gold->bfsFromEGold());
}

TEST_F(DAGThreeF, breadthFirstSearch_FromF)
{
  auto const tree = alg::breadthFirstSearch(gold->graph(), gold->f());
  EXPECT_EQ(tree, gold->bfsFromFGold());
}
