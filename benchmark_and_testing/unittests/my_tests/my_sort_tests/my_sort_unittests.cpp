// Unit test utils
//#include <predefined_utils/testing/fixtures/sort_testing_fixtures.h>

// Day 2 sort library
//#include <lib3611/w1d1_2_sort/counting_sort.h>
//#include <lib3611/w1d1_2_sort/binary_sort.h>
#include <lib3611/w1d1_2_sort/radix_sort.h>
//#include <lib3611/w1d1_2_sort/custom_aa_sort.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <ranges>
#include <algorithm>

namespace alg = dte3611::sort::algorithms;

struct MyCustomSortTestF : ::testing::Test {

  using ::testing::Test::Test;
  ~MyCustomSortTestF() override {}

  struct Item {
    int age;
  };

  using ItemVector = std::vector<Item>;
  ItemVector m_data;

  void SetUp() final
  {
    m_data.push_back({3});
    m_data.push_back({1});
    m_data.push_back({5});
    m_data.push_back({4});
  }
  void TearDown() final { m_data.clear(); }
};

struct MyCustomRadixSortTestF : ::testing::Test {

  using ::testing::Test::Test;
  ~MyCustomRadixSortTestF() override {}

  using ItemVector = std::vector<int>;
  ItemVector m_data, m_gold;

  void SetUp() final
  {
    m_data.push_back({333});
    m_data.push_back({11});
    m_data.push_back({5});
    m_data.push_back({44});

    m_gold.push_back({5});
    m_gold.push_back({11});
    m_gold.push_back({44});
    m_gold.push_back({333});
  }
  void TearDown() final { m_data.clear(); }
};


// Test on empty dataset
TEST_F(MyCustomSortTestF, std_ranges_sort_test)
{
  std::ranges::sort(m_data, {}, &Item::age);

  auto const mdata_size = std::ssize(m_data);
  for (auto i = 0; i < mdata_size - 1; ++i)
    EXPECT_GE(m_data[i + 1].age, m_data[i].age);
}

TEST_F(MyCustomRadixSortTestF, my_custom_radix_sort_test)
{
  alg::radix_sort(m_data);
  EXPECT_EQ(m_data, m_gold);
}
