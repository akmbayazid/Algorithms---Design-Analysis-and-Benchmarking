#ifndef DTE3611_PREDEF_TESTING_STRING_MATCH_GOLD_H
#define DTE3611_PREDEF_TESTING_STRING_MATCH_GOLD_H

// gtest
#include <gtest/gtest.h>

// stl
#include <concepts>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <random>
#include <optional>

namespace dte3611::predef::testing::string_match
{

  namespace detail
  {
    using StdStringGoldPair
      = std::pair<std::string, std::optional<std::string::difference_type>>;
    using StdStringGoldPairs = std::vector<StdStringGoldPair>;

    namespace generators
    {

      template <typename Generator_T>
      concept CollectionGenerator = requires(Generator_T g, std::string s)
      {
        typename Generator_T::GoldPairs;

        {
          g(s)
          } -> std::convertible_to<typename Generator_T::GoldPairs>;
      };

      struct EmptySMColGenerator {
        using GoldPairs = StdStringGoldPairs;

        GoldPairs operator()(std::string& the_empty_string) const
        {
          the_empty_string = std::string{};
          GoldPairs sgps;
          sgps.emplace_back("hello", std::nullopt);
          sgps.emplace_back("world", std::nullopt);
          sgps.emplace_back("", 0);
          return sgps;
        }
      };

      struct HelloWorldSMColGenerator {
        using GoldPairs = StdStringGoldPairs;

        GoldPairs operator()(std::string& the_string) const
        {
          the_string = "Hello world !_!";
          GoldPairs sgps;
          sgps.emplace_back("hello", std::nullopt);
          sgps.emplace_back("world", 6);
          sgps.emplace_back("", 0);
          sgps.emplace_back("_!", 13);
          return sgps;
        }
      };

    }   // namespace generators

    template <generators::CollectionGenerator Generator_T>
    class CollectionTemplate {
    public:
      using GoldPairs = StdStringGoldPairs;
      using Generator = Generator_T;

      CollectionTemplate()  = default;
      ~CollectionTemplate() = default;

      void init() { m_sg_pairs = m_gen(m_string); }

      std::string const& string() const { return m_string; }
      GoldPairs const&   sequenceGoldPairs() const { return m_sg_pairs; }

    private:
      Generator    m_gen;
      std::string  m_string;
      GoldPairs    m_sg_pairs;
    };

  }   // namespace detail

  namespace gold
  {

    struct Empty
      : detail::CollectionTemplate<detail::generators::EmptySMColGenerator> {

      Empty() { init(); }
    };

    struct HelloWorld : detail::CollectionTemplate<
                          detail::generators::HelloWorldSMColGenerator> {

      HelloWorld() { init(); }
    };

  }   // namespace gold

}   // namespace dte3611::predef::testing::string_match

#endif   // DTE3611_PREDEF_TESTING_STRING_MATCH_GOLD_H
