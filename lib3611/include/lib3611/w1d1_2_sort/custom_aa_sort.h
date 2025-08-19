#ifndef DTE3611_WEEK1_CUSTOM_AA_SORT_H
#define DTE3611_WEEK1_CUSTOM_AA_SORT_H

// stl
#include <iterator>
#include <algorithm>

namespace dte3611::sort::algorithms
{

  namespace detail
  {

    struct custom_aa_sort_fn {

      /**************************
       *  Iterator Range Operator
       */

      // Type Generics
      template <std::random_access_iterator   Iterator_T,
                std::sentinel_for<Iterator_T> Sentinel_T,
                typename Compare_T    = std::ranges::less,
                typename Projection_T = std::identity>
      // Algorithm type requirements
      requires std::sortable<Iterator_T, Compare_T, Projection_T>
      // Return value
      constexpr Iterator_T
      // Call-operator signature
      operator()(Iterator_T /*first*/, Sentinel_T /*last*/,
                 Compare_T /*comp*/ = {}, Projection_T /*proj*/ = {}) const
      {
        return {};
      }

      /******************
       *  Ranges Operator
       */

      // Type Generics
      template <std::ranges::random_access_range Range_T,
                typename Compare_T    = std::ranges::less,
                typename Projection_T = std::identity>
      // Algorithm type requirements
      requires std::sortable<std::ranges::iterator_t<Range_T>, Compare_T,
                             Projection_T>
      // Return value
      constexpr std::ranges::borrowed_iterator_t<Range_T>
      // Call-operator signature
      operator()(Range_T&& range, Compare_T comp = {},
                 Projection_T proj = {}) const
      {
        return (*this)(std::ranges::begin(range), std::ranges::end(range),
                       std::move(comp), std::move(proj));
//        static_assert(false, "Complete the code"
//                             "- find the appropriate call signature in the "
//                             "cpp reference documentation.");
      }
    };

  }   // namespace detail

  // Niebloid API Instantiation
  inline constexpr detail::custom_aa_sort_fn custom_aa_sort{};

}   // namespace dte3611::sort::algorithms

#endif   // DTE3611_WEEK1_CUSTOM_AA_SORT_H
