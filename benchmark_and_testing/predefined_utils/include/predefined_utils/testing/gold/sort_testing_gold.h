#ifndef DTE3611_PREDEF_TESTING_SORT_GOLD_H
#define DTE3611_PREDEF_TESTING_SORT_GOLD_H

// gtest
#include <gtest/gtest.h>

// stl
#include <concepts>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <random>

namespace dte3611::predef::testing::sort
{

  namespace detail
  {

    namespace generators
    {

      template <typename Generator_T>
      concept CollectionGenerator = requires(Generator_T g, size_t no_e)
      {
        typename Generator_T::Element;
        typename Generator_T::Container;

        {
          g(no_e)
          } -> std::convertible_to<typename Generator_T::Container>;
      };

      struct EmptyIntColGenerator {
        using Element   = int;
        using Container = std::vector<Element>;

        Container operator()(size_t /*no_elements*/) const { return {}; }
      };

      struct SortedIntColGenerator {
        using Element   = int;
        using Container = std::vector<Element>;

        Container operator()(size_t no_elements) const
        {
          Container c;
          c.reserve(no_elements);
          for (auto e = 0ul; e < no_elements; ++e) c.emplace_back(e);
          return c;
        }
      };


      struct ReverseIntColGenerator {
        using Element   = int;
        using Container = std::vector<Element>;

        Container operator()(size_t no_elements) const
        {
          Container c;
          c.reserve(no_elements);
          for (auto e = 0ul; e < no_elements; ++e) c.emplace_back(e);
          std::reverse(std::begin(c), std::end(c));
          return c;
        }
      };

      struct OrganpipeIntColGenerator {
        using Element   = int;
        using Container = std::vector<Element>;

        Container operator()(size_t no_elements) const
        {
          Container c;
          c.reserve(no_elements);
          for (auto e = 0ul; e <= no_elements / 2; ++e) c.emplace_back(e);
          for (auto e = Element(no_elements / 2 - 1); e > 0; --e)
            c.emplace_back(e);
          c.emplace_back(0);
          return c;
        }
      };

      struct RotatedIntColGenerator {
        using Element   = int;
        using Container = std::vector<Element>;

        Container operator()(size_t no_elements) const
        {
          Container c;
          c.reserve(no_elements);
          for (auto e = 0ul; e < no_elements; ++e) c.emplace_back(e);
          std::rotate(std::begin(c), std::begin(c) + 1, std::end(c));
          return c;
        }
      };

      struct Random01IntColGenerator {
        using Element   = int;
        using Container = std::vector<Element>;

        Container operator()(size_t no_elements) const
        {
          Container c;
          c.reserve(no_elements);

          std::random_device rd;   // Will be used to obtain a seed for the
                                   // random number engine
          std::mt19937 gen(
            rd());   // Standard mersenne_twister_engine seeded with rd()
          std::uniform_int_distribution<> distrib(0, 1);
          for (auto e = 0ul; e < no_elements; ++e) c.emplace_back(distrib(gen));
          return std::move(c);
        }
      };

    }   // namespace generators


    template <generators::CollectionGenerator Generator_T>
    class CollectionTemplate {
    public:
      using Generator = Generator_T;
      using Container = typename Generator::Container;

      CollectionTemplate()  = default;
      ~CollectionTemplate() = default;

      template <typename Comp_T = std::less<typename Generator::Element>>
      void init(size_t no_elements, Comp_T /*comp*/ = Comp_T())
      {
        m_gold = m_data = m_gen(no_elements);
        std::sort(std::begin(m_gold), std::end(m_gold));
      }

      Container const& collection() const { return m_data; }
      Container const& gold() const { return m_gold; }

    private:
      Generator m_gen;
      Container m_data;
      Container m_gold;
    };

  }   // namespace detail

  namespace gold
  {

    struct IntsEmpty
      : detail::CollectionTemplate<detail::generators::EmptyIntColGenerator> {

      IntsEmpty() { init(10); }
    };

    struct SortedIntsSmall
      : detail::CollectionTemplate<detail::generators::SortedIntColGenerator> {

      SortedIntsSmall() { init(10); }
    };

    struct ReverseIntsSmall
      : detail::CollectionTemplate<detail::generators::ReverseIntColGenerator> {

      ReverseIntsSmall() { init(10); }
    };

    struct OrganpipeIntsSmall
      : detail::CollectionTemplate<
          detail::generators::OrganpipeIntColGenerator> {

      OrganpipeIntsSmall() { init(10); }
    };

    struct RotatedIntsSmall
      : detail::CollectionTemplate<detail::generators::RotatedIntColGenerator> {

      RotatedIntsSmall() { init(10); }
    };

    struct Random01IntsSmall : detail::CollectionTemplate<
                                 detail::generators::Random01IntColGenerator> {

      Random01IntsSmall() { init(10); }
    };
  }   // namespace gold

}   // namespace dte3611::predef::testing::sort

#endif   // DTE3611_PREDEF_TESTING_SORT_GOLD_H
