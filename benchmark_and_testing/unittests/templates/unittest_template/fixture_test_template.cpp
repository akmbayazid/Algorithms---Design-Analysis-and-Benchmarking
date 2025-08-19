// Unit test utils
//#include <predef_utils/ ... .h>

// Library to test headers
// #include <lib3611/ ... .h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
// #include <vector>


// Example function to test
auto passThrough( auto const& p_value ) { return p_value; }


// Template fixture struct
struct TestTemplateFixture : ::testing::Test {

  using ::testing::Test::Test;
  ~TestTemplateFixture() override {}

  std::unique_ptr<int> data;

  void SetUp() final {
    data = std::make_unique<int>();
    *data = 1;
  }
  void TearDown() final { data.release(); }
};


// Test on empty dataset
TEST_F(TestTemplateFixture, isDataOne)
{
  auto const expected_gold = 1;
  auto const value         = passThrough(*data);
  EXPECT_EQ(value, expected_gold);
}


// Test on empty dataset
TEST_F(TestTemplateFixture, isDataPositive)
{
  auto const value         = passThrough(*data);
  EXPECT_GE(value, 0);
}
