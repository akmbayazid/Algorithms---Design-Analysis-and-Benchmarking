// Unit test utils
//#include <predef_utils/ ... .h>

// Library to test headers
// #include <lib3611/ ... .h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
// #include <vector>



// Example function to test
auto passThroughDummyFunc( auto const& p_value ) { return p_value; }


// Test on empty dataset
TEST(TestTemplateSuite, isDataOne)
{
  auto const expected_gold = 1ul;
  auto const value         = passThroughDummyFunc(1);
  EXPECT_EQ(value, expected_gold);
}


// Test on empty dataset
TEST(TestTemplateSuite, isDataPositive)
{
  auto const value         = passThroughDummyFunc(1);
  EXPECT_GE(value, 0);
}
