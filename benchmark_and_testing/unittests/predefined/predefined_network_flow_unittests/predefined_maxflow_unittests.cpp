// Unit test utils
#include <predefined_utils/testing/fixtures/flow_testing_fixtures.h>

// Week 3 network flow library
#include <lib3611/w3_network_flow/network_flow.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>

// Qualify predefined fixtures
using namespace dte3611::predef::testing::flow::fixtures;

namespace alg = dte3611::np::algorithms;

TEST_F(FordFulkersonDemoDAGF, FordFulkersonMaxFlowDemo)
{
  auto max_flow = alg::maxFlow(gold->graph(), gold->s(), gold->t());
  auto gold_flow = gold->maxFlowSTGold();
  EXPECT_EQ(max_flow, gold_flow);
}

TEST_F(FordFulkersonDemoDAG2F, FordFulkersonMaxFlowDemo2)
{
  auto max_flow = alg::maxFlow(gold->graph(), gold->s(), gold->t());
  auto gold_flow = gold->maxFlowSTGold();
  EXPECT_EQ(max_flow, gold_flow);
}

TEST_F(MinCostMaxFlowDAG, FordFulkersonMinCostFlowDag)
{
  auto max_flow = alg::maxFlow(gold->graph(), gold->s(), gold->t());
  auto gold_flow = gold->maxFlowSTGold();
  EXPECT_EQ(max_flow, gold_flow);
}

