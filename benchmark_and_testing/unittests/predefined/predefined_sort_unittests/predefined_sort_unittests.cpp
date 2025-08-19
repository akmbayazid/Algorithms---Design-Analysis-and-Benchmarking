// Unit test utils
#include <predefined_utils/testing/fixtures/sort_testing_fixtures.h>

// Day 2 sort library
#include <lib3611/w1d1_2_sort/counting_sort.h>
#include <lib3611/w1d1_2_sort/binary_sort.h>
#include <lib3611/w1d1_2_sort/radix_sort.h>
#include <lib3611/w1d1_2_sort/custom_aa_sort.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>

// Qualify predefined fixtures
using namespace dte3611::predef::testing::sort::fixtures;

namespace alg = dte3611::sort::algorithms;


// Test on empty dataset
TEST_F(EmptyIntsCollF, countingSort)
{
  auto collection = data->collection();
  alg::counting_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(EmptyIntsCollF, binarySort)
{
  auto collection = data->collection();
  alg::binary_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(EmptyIntsCollF, radixSort)
{
  auto collection = data->collection();
  alg::radix_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(EmptyIntsCollF, customAaSort)
{
  auto collection = data->collection();
  alg::custom_aa_sort(collection);
  EXPECT_EQ(collection, data->gold());
}


// Test on pre sorted dataset
TEST_F(SmallSortedIntsCollF, countingSort)
{
  auto collection = data->collection();
  alg::counting_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallSortedIntsCollF, binarySort)
{
  auto collection = data->collection();
  alg::binary_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallSortedIntsCollF, radixSort)
{
  auto collection = data->collection();
  alg::radix_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallSortedIntsCollF, customAaSort)
{
  auto collection = data->collection();
  alg::custom_aa_sort(collection);
  EXPECT_EQ(collection, data->gold());
}


// Test on reverse pre sorted dataset
TEST_F(SmallReverseIntsCollF, countingSort)
{
  auto collection = data->collection();
  alg::counting_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallReverseIntsCollF, binarySort)
{
  auto collection = data->collection();
  alg::binary_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallReverseIntsCollF, radixSort)
{
  auto collection = data->collection();
  alg::radix_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallReverseIntsCollF, customAaSort)
{
  auto collection = data->collection();
  alg::custom_aa_sort(collection);
  EXPECT_EQ(collection, data->gold());
}


// Test on organpipe dataset
TEST_F(SmallOrganpipeIntsCollF, countingSort)
{
  auto collection = data->collection();
  alg::counting_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallOrganpipeIntsCollF, binarySort)
{
  auto collection = data->collection();
  alg::binary_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallOrganpipeIntsCollF, radixSort)
{
  auto collection = data->collection();
  alg::radix_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallOrganpipeIntsCollF, customAaSort)
{
  auto collection = data->collection();
  alg::custom_aa_sort(collection);
  EXPECT_EQ(collection, data->gold());
}


// Test on rotated dataset
TEST_F(SmallRotatedIntsCollF, countingSort)
{
  auto collection = data->collection();
  alg::counting_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallRotatedIntsCollF, binarySort)
{
  auto collection = data->collection();
  alg::binary_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallRotatedIntsCollF, radixSort)
{
  auto collection = data->collection();
  alg::radix_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallRotatedIntsCollF, customAaSort)
{
  auto collection = data->collection();
  alg::custom_aa_sort(collection);
  EXPECT_EQ(collection, data->gold());
}


// Test on rotated dataset
TEST_F(SmallRandom01IntsCollF, countingSort)
{
  auto collection = data->collection();
  alg::counting_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallRandom01IntsCollF, binarySort)
{
  auto collection = data->collection();
  alg::binary_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallRandom01IntsCollF, radixSort)
{
  auto collection = data->collection();
  alg::radix_sort(collection);
  EXPECT_EQ(collection, data->gold());
}

TEST_F(SmallRandom01IntsCollF, customAaSort)
{
  auto collection = data->collection();
  alg::custom_aa_sort(collection);
  EXPECT_EQ(collection, data->gold());
}


// Test on custom element dataset
//TEST_F(GoldPeopleSortTestF, countingSortByAge)
//{
//  auto data = m_people;
//  alg::counting_sort(data, {}, &detail::Person::age);

//  EXPECT_EQ(data, m_gold_age);
//}

//TEST_F(GoldPeopleSortTestF, countingSortByName)
//{
//  auto data = m_people;
//  alg::counting_sort(data, {}, &detail::Person::name);

//  EXPECT_EQ(data, m_gold_name);
//}

TEST_F(GoldPeopleSortTestF, binarySortByAge)
{
  auto data = m_people;
  alg::binary_sort(data, {}, &detail::Person::age);

  EXPECT_EQ(data, m_gold_age);
}

TEST_F(GoldPeopleSortTestF, binarySortByName)
{
  auto data = m_people;
  alg::binary_sort(data, {}, &detail::Person::name);

  EXPECT_EQ(data, m_gold_name);
}

TEST_F(GoldPeopleSortTestF, radixSortByAge)
{
  auto data = m_people;
  alg::radix_sort(data, {}, &detail::Person::age);

  EXPECT_EQ(data, m_gold_age);
}

TEST_F(GoldPeopleSortTestF, radixSortByName)
{
  auto data = m_people;
  alg::radix_sort(data, {}, &detail::Person::name);

  EXPECT_EQ(data, m_gold_name);
}

TEST_F(GoldPeopleSortTestF, customAaSortByAge)
{
  auto data = m_people;
  alg::custom_aa_sort(data, std::less<>(), &detail::Person::age);

  EXPECT_EQ(data, m_gold_age);
}

TEST_F(GoldPeopleSortTestF, customAaSortByName)
{
  auto data = m_people;
  alg::custom_aa_sort(data, {}, &detail::Person::name);

  EXPECT_EQ(data, m_gold_name);
}

// Test on custom element dataset and custom predicate
//TEST_F(GoldPeopleSortTestF, countingSortByAgeGreater)
//{
//  auto data = m_people;
//  alg::counting_sort(data, std::greater(), &detail::Person::age);

//  EXPECT_EQ(data, m_gold_age_greater);
//}

TEST_F(GoldPeopleSortTestF, binarySortByAgeGreater)
{
  auto data = m_people;
  alg::binary_sort(data, std::greater(), &detail::Person::age);

  EXPECT_EQ(data, m_gold_age_greater);
}

TEST_F(GoldPeopleSortTestF, radixSortByAgeGreater)
{
  auto data = m_people;
  alg::radix_sort(data, std::greater(), &detail::Person::age);

  EXPECT_EQ(data, m_gold_age_greater);
}

TEST_F(GoldPeopleSortTestF, customAaSortByAgeGreater)
{
  auto data = m_people;
  alg::custom_aa_sort(data, std::greater(), &detail::Person::age);

  EXPECT_EQ(data, m_gold_age_greater);
}
