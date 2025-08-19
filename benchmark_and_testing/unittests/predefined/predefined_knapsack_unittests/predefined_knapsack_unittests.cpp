// Unit test utils
//#include <predefined_utils/testing/fixtures/sort_testing_fixtures.h>

// Day 2 sort library
#include <lib3611/w2_knapsack/knapsack.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <ranges>
#include <algorithm>
#include <compare>

namespace detail {
  struct Item {
    std::string name;
    int         value;
    int         weight;

    std::weak_ordering operator <=> ( Item const& other ) const = default;
  };
}

using namespace detail;

// Qualify predefined fixtures
//using namespace dte3611::predef::testing::sort::fixtures;
struct KnapsackFixture01 : ::testing::Test {

  using ::testing::Test::Test;
  ~KnapsackFixture01() override {}

  using Items    = std::vector<detail::Item>;
  using Capacity = int;
  using K01Gold  = std::vector<bool>;
  using K01GoldV = int;

  using TestSet  = std::tuple<Items, Capacity, K01Gold, K01GoldV>;
  using TestSets = std::vector<TestSet>;


  TestSets test_sets;

  void SetUp() final
  {
    auto const A = Item{"A", 24, 12};
    auto const B = Item{"B", 13,  7};
    auto const C = Item{"C", 23, 11};
    auto const D = Item{"D", 15,  8};
    auto const E = Item{"E", 16,  9};

    test_sets.push_back(
      {{A, B, C, D, E}, 26, {false, true, true, true, false}, 51});

    auto const R = Item{"R", 1, 1};
    auto const S = Item{"R", 6, 2};
    auto const T = Item{"T", 18, 5};
    auto const U = Item{"U", 22, 6};
    auto const V = Item{"V", 28, 7};

    test_sets.push_back(
      {{R, S, T, U, V}, 11, {false, false, true, true, false}, 40});
  }

  void TearDown() final
  {
    test_sets.clear();
  }
};

namespace alg = dte3611::np::algorithms;


// Test knapsack algorithm on
TEST_F(KnapsackFixture01, firstTestSetFoundByAGoogleSearch)
{
  for (auto const& [items, capacity, gold, gold_v] : test_sets) {
    std::vector<bool>     my_01knapsack;
    [[maybe_unused]] auto result = alg::knapsack_01(
      items, std::back_inserter(my_01knapsack), capacity,
      &detail::Item::value, &detail::Item::weight);

    std::reverse(my_01knapsack.begin(), my_01knapsack.end());

//    EXPECT_EQ(result, not gold.empty());
    EXPECT_EQ(result, gold_v);
    EXPECT_EQ(std::ssize(my_01knapsack), std::ssize(gold));
    EXPECT_EQ(my_01knapsack, gold);
  }
}
