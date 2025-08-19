// Unit test utils
//#include <predefined_utils/testing/fixtures/sort_testing_fixtures.h>

// Day 2 sort library
#include <lib3611/w2_subset_sum/subset_sum.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <ranges>
#include <algorithm>

// Qualify predefined fixtures
//using namespace dte3611::predef::testing::sort::fixtures;
struct SubsetSumFixture01 : ::testing::Test {

  using ::testing::Test::Test;
  ~SubsetSumFixture01() override {}

  using ValueType = int;
  using TargetType = ValueType;

  using NumberVector = std::vector<ValueType>;
  using TargetNumberVectorsPair
    = std::pair<TargetType, std::vector<NumberVector>>;
  using TargetResultVector = std::vector<TargetNumberVectorsPair>;


  NumberVector       numbers;
  TargetResultVector gold;

  void SetUp() final
  {
    numbers = {1, 2, 3, 4};
    gold.push_back({7, {{1, 2, 4}, {3, 4}}});
    gold.push_back({11, {}});
//    for (auto& g : gold) std::ranges::sort(g.second);
  }

  void TearDown() final
  {
    numbers.clear();
    gold.clear();
  }
};

namespace alg = dte3611::np::algorithms;


// Test subsetsum algorithm on first dataset found by a google search
TEST_F(SubsetSumFixture01, firstTestSetFoundByAGoogleSearch)
{
  for (auto const& [target, gold_result] : gold) {
    auto result = alg::subset_sum(numbers, target, {});
    std::ranges::sort(result);
    EXPECT_EQ(std::ssize(result), std::ssize(gold_result));
    EXPECT_EQ(result, gold_result);
  }
}
