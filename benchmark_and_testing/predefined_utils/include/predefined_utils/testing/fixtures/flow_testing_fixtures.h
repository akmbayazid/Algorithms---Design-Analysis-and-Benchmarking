#ifndef DTE3611_PREDEF_TESTING_FLOW_FIXTURES_H
#define DTE3611_PREDEF_TESTING_FLOW_FIXTURES_H

#include "../gold/flow_testing_gold.h"

// gtest
#include <gtest/gtest.h>

namespace dte3611::predef::testing::flow::fixtures
{

  template <typename GoldGraph_T>
  struct GoldGraphTestF : ::testing::Test {

    using GoldGraph = GoldGraph_T;

    using ::testing::Test::Test;
    ~GoldGraphTestF() override {}

    std::unique_ptr<GoldGraph> gold;

    void SetUp() final { gold = std::make_unique<GoldGraph>(); }
    void TearDown() final { gold.release(); }
  };

  // Ford-Fulkerson testing fixtures
  using FordFulkersonDemoDAGF = GoldGraphTestF<gold::FordFulkersonDemoDAG>;
  using FordFulkersonDemoDAG2F = GoldGraphTestF<gold::FordFulkersonDemoDAG2>;

  // Min-cost max-flow testing fixtures
  using MinCostMaxFlowDAG = GoldGraphTestF<gold::MinCostMaxFlowDAG>;

}   // namespace dte3611::predef::testing::graph::fixtures

#endif   // DTE3611_PREDEF_TESTING_FLOW_FIXTURES_H
