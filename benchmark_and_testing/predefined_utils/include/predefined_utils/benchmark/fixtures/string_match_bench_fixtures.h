#ifndef DTE3611_PREDEF_BENCHMARKING_STRING_MATCH_FIXTURES_H
#define DTE3611_PREDEF_BENCHMARKING_STRING_MATCH_FIXTURES_H

// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>

namespace dte3611::predef::benchmarking::string_match::fixtures
{

  namespace detail
  {

    struct StringMatchBenchmarkFixtureTemplate : benchmark::Fixture {

      using benchmark::Fixture::Fixture;
      ~StringMatchBenchmarkFixtureTemplate() override {}

      std::string m_string;
      std::string m_sequence;

      void TearDown(const benchmark::State&) override {}
    };
  }   // namespace detail

  struct HelloWorldF : detail::StringMatchBenchmarkFixtureTemplate {
    using Base = detail::StringMatchBenchmarkFixtureTemplate;

    using Base::Base;
    ~HelloWorldF() override {}

    void SetUp(const benchmark::State& /*st*/) final
    {
      m_string   = "Hello world !_!";
      m_sequence = "!_!";
    }
  };

}   // namespace dte3611::predef::benchmarking::string_match::fixtures

#endif   // DTE3611_PREDEF_BENCHMARKING_STRING_MATCH_FIXTURES_H
