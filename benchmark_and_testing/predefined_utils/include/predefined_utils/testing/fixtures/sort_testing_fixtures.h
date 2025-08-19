#ifndef DTE3611_PREDEF_TESTING_SORT_FIXTURES_H
#define DTE3611_PREDEF_TESTING_SORT_FIXTURES_H

#include "../gold/sort_testing_gold.h"

// gtest
#include <gtest/gtest.h>

namespace dte3611::predef::testing::sort::fixtures
{

  namespace detail {

    template <typename GoldSortCollection_T>
    struct GoldSortTestF : ::testing::Test {

      using GoldSortCollection = GoldSortCollection_T;

      using ::testing::Test::Test;
      ~GoldSortTestF() override {}

      std::unique_ptr<GoldSortCollection> data;

      void SetUp() final { data = std::make_unique<GoldSortCollection>(); }
      void TearDown() final { data.release(); }
    };

    struct Person {
      std::string name;
      size_t      age;
      auto        operator<=>(Person const&) const = default;
    };

  }   // namespace detail

  using EmptyIntsCollF          = detail::GoldSortTestF<gold::IntsEmpty>;
  using SmallSortedIntsCollF    = detail::GoldSortTestF<gold::SortedIntsSmall>;
  using SmallReverseIntsCollF   = detail::GoldSortTestF<gold::ReverseIntsSmall>;
  using SmallOrganpipeIntsCollF = detail::GoldSortTestF<gold::OrganpipeIntsSmall>;
  using SmallRotatedIntsCollF   = detail::GoldSortTestF<gold::RotatedIntsSmall>;
  using SmallRandom01IntsCollF  = detail::GoldSortTestF<gold::Random01IntsSmall>;

  struct GoldPeopleSortTestF : ::testing::Test {

    using ::testing::Test::Test;
    ~GoldPeopleSortTestF() override {}

    using PeopleVec = std::vector<detail::Person>;
    PeopleVec m_people;
    PeopleVec m_gold_name;
    PeopleVec m_gold_age;
    PeopleVec m_gold_age_greater;

    template <typename MembType_T, typename Compare_T = std::less<MembType_T>>
    struct MembCompareOp {
      MembType_T detail::Person::*m_memb_name;
      Compare_T m_comp;

      MembCompareOp(MembType_T detail::Person::*memb_name,
                    Compare_T                   comp = Compare_T())
        : m_memb_name{memb_name}, m_comp{comp}
      {
      }

      bool operator()(detail::Person const& lhs,
                      detail::Person const& rhs) const
      {
        return m_comp(lhs.*m_memb_name, rhs.*m_memb_name);
      }
    };



    void SetUp() final
    {
      m_people.emplace_back(detail::Person{.name = "Kari Normann", .age = 84u});
      m_people.emplace_back(detail::Person{.name = "Ola Normann", .age = 80u});
      m_people.emplace_back(detail::Person{.name = "John Doe", .age = 76u});
      m_people.emplace_back(detail::Person{.name = "Jane Doe", .age = 81u});

      m_gold_name = m_gold_age = m_gold_age_greater = m_people;

      std::ranges::sort(m_gold_name.begin(), m_gold_name.end(),
                        {}, &detail::Person::name);

      std::ranges::sort(m_gold_age.begin(), m_gold_age.end(),
                        MembCompareOp(&detail::Person::age));

      std::ranges::sort(m_gold_age_greater.begin(), m_gold_age_greater.end(),
                        MembCompareOp(&detail::Person::age, std::greater<>()));
    }

    void TearDown() final {
      m_people.clear();
      m_gold_name.clear();
      m_gold_age.clear();
    }

  };


}   // namespace dte3611::predef::testing::sort::fixtures

#endif   // DTE3611_PREDEF_TESTING_SORT_FIXTURES_H
