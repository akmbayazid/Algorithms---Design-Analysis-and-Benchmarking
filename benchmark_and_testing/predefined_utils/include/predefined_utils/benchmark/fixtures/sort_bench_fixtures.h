#ifndef DTE3611_PREDEF_BENCHMARKING_SORT_FIXTURES_H
#define DTE3611_PREDEF_BENCHMARKING_SORT_FIXTURES_H

// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>

namespace dte3611::predef::benchmarking::sort::fixtures
{

  namespace detail
  {

    template <typename Element_T, typename Comp_T = std::less<Element_T>>
    struct SortBenchmarkFixtureTemplate : benchmark::Fixture {

      using benchmark::Fixture::Fixture;
      ~SortBenchmarkFixtureTemplate() override {}

      using Element = Element_T;
      using Vector  = std::vector<Element>;

      Vector m_data;

      void TearDown(const benchmark::State&) override { m_data.clear(); }
    };

  }   // namespace detail

  struct SortedIntColF : detail::SortBenchmarkFixtureTemplate<int> {
    using Base = detail::SortBenchmarkFixtureTemplate<int>;

    using Base::Base;
    ~SortedIntColF() override {}

    void SetUp(const benchmark::State& st) final
    {
      auto const no_elements = st.range(0);
      m_data.reserve(no_elements);
      for (auto e = 0; e < no_elements; ++e) m_data.emplace_back(e);
    }
  };

  struct ReverseIntColF : detail::SortBenchmarkFixtureTemplate<int> {
    using Base = detail::SortBenchmarkFixtureTemplate<int>;

    using Base::Base;
    ~ReverseIntColF() override {}

    void SetUp(const benchmark::State& st) final
    {
      auto const no_elements = st.range(0);
      m_data.reserve(no_elements);
      for (auto e = 0; e < no_elements; ++e)
        m_data.emplace_back(no_elements - e - 1);
    }
  };

  struct OrganpipeIntColF : detail::SortBenchmarkFixtureTemplate<int> {
    using Base = detail::SortBenchmarkFixtureTemplate<int>;

    using Base::Base;
    ~OrganpipeIntColF() override {}

    void SetUp(const benchmark::State& st) final
    {
      auto const no_elements = st.range(0);
      m_data.reserve(no_elements);
      for (auto e = 0ul; e <= no_elements / 2; ++e) m_data.emplace_back(e);
      for (auto e = size_t(no_elements / 2 - 1); e > 0; --e)
        m_data.emplace_back(e);
      m_data.emplace_back(0);
    }
  };

  struct RotatedIntColF : detail::SortBenchmarkFixtureTemplate<int> {
    using Base = detail::SortBenchmarkFixtureTemplate<int>;

    using Base::Base;
    ~RotatedIntColF() override {}

    void SetUp(const benchmark::State& st) final
    {
      auto const no_elements = st.range(0);
      m_data.reserve(no_elements);
      for (auto e = 0ul; e < no_elements; ++e) m_data.emplace_back(e);
      std::rotate(m_data.begin(), m_data.begin() + 1, m_data.end());
    }
  };

  struct Random01IntColF : detail::SortBenchmarkFixtureTemplate<int> {
    using Base = detail::SortBenchmarkFixtureTemplate<int>;

    using Base::Base;
    ~Random01IntColF() override {}

    void SetUp(const benchmark::State& st) final
    {
      auto const no_elements = st.range(0);
      m_data.reserve(no_elements);
      std::random_device rd;   // Will be used to obtain a seed for the
                               // random number engine
      std::mt19937 gen(
        rd());   // Standard mersenne_twister_engine seeded with rd()
      std::uniform_int_distribution<> distrib(0, 1);
      for (auto e = 0ul; e < no_elements; ++e)
        m_data.emplace_back(distrib(gen));
    }
  };

}   // namespace dte3611::predef::benchmarking::sort::fixtures




#endif   // DTE3611_PREDEF_BENCHMARKING_SORT_FIXTURES_H
