// Unit test utils
#include <predefined_utils/testing/fixtures/string_match_testing_fixtures.h>

// Day3 string match library
#include <lib3611/w1d3_string_match/naive_search.h>
#include <lib3611/w1d3_string_match/kmp_search.h>
#include <lib3611/w1d3_string_match/bmh_search.h>
#include <lib3611/w1d3_string_match/kr_search.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <string_view>
using namespace std::literals::string_view_literals;

// Qualify predefined fixtures
using namespace dte3611::predef::testing::string_match::fixtures;

namespace alg = dte3611::string_match::algorithms;


// Test on empty dataset
TEST_F(EmptySMCollF, naiveSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::naive_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(EmptySMCollF, kmpSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::kmp_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(EmptySMCollF, bmhSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::bmh_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(EmptySMCollF, krSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::kr_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}


// Test on HelloWorld dataset
TEST_F(HelloWorldSMCollF, naiveSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::naive_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(HelloWorldSMCollF, kmpSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::kmp_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(HelloWorldSMCollF, bmhSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::bmh_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(HelloWorldSMCollF, krSearch)
{
  auto const& string = data->string();
  auto const& sgps   = data->sequenceGoldPairs();

  for (auto const& [sequence, gold] : sgps) {

    auto const res        = alg::kr_search(string, sequence);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}


// Test on a custom alphabet
TEST_F(GoldCustomAlphabetStringMatchTestF, naiveSearch)
{
  auto const& string = m_string;
  auto const& sgps   = m_sg_pairs;

  for (auto const& [sequence, gold] : sgps) {

    auto const res
      = alg::naive_search(string, sequence, {}, Char_uPtr_proj, Char_uPtr_proj);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold) {
      EXPECT_EQ(res, sequence.empty() ? string.begin() : string.end());
    }
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(GoldCustomAlphabetStringMatchTestF, kmpSearch)
{
  auto const& string = m_string;
  auto const& sgps   = m_sg_pairs;

  for (auto const& [sequence, gold] : sgps) {

    auto const res
      = alg::kmp_search(string, sequence, {}, Char_uPtr_proj, Char_uPtr_proj);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, sequence.empty() ? string.begin() : string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(GoldCustomAlphabetStringMatchTestF, bmhSearch)
{
  auto const& string = m_string;
  auto const& sgps   = m_sg_pairs;


  for (auto const& [sequence, gold] : sgps) {

    auto const res
      = alg::bmh_search(string, sequence, {}, Char_uPtr_proj, Char_uPtr_proj);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}

TEST_F(GoldCustomAlphabetStringMatchTestF, krSearch)
{
  auto const& string = m_string;
  auto const& sgps   = m_sg_pairs;

  for (auto const& [sequence, gold] : sgps) {

    auto const res
      = alg::kr_search(string, sequence, {}, Char_uPtr_proj, Char_uPtr_proj);
    auto const res_offset = std::ranges::distance(string.begin(), res);

    if (not gold)
      EXPECT_EQ(res, string.end());
    else
      EXPECT_EQ(res_offset, gold);
  }
}
