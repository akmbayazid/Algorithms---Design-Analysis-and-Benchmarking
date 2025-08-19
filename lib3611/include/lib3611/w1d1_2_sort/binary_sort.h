#ifndef DTE3611_WEEK1_BINARY_SORT_H
#define DTE3611_WEEK1_BINARY_SORT_H

// stl
#include <iterator>
#include <ranges>
#include <algorithm>
#include <set>
#include <functional>

namespace dte3611::sort::algorithms
{

  namespace detail
  {

    struct binary_sort_fn {

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
      operator()(Iterator_T first, Sentinel_T last,
                 Compare_T comp = {}, Projection_T proj = {}) const
      {
          // 1 use std:multiset (binary tree)
          //1 b implement a custom comparator ( as a lambda)
          // 2 copy range [first t, last] to the multiset
          // 3 copy back from multiset and write back result

          auto const count = std::ranges::distance(first, last);
          if ( count <=1) return last;

          //Determine the type of the values that the iterator points to
          using value_type = typename std::iterator_traits<Iterator_T>::value_type;

            // use the specified comparator and invoke the specified
          //projection to the iterator value
          using val = std::iter_value_t<Iterator_T>;

          // creat a comparison function object (as lambda)
          auto c = [&] (const val & a, const val&b){
              return comp(std::invoke(proj, a),std::invoke(proj, b));
          };

        using ms = std::multiset<value_type, decltype(c)>;
          ms btree(c);
            // copy element to the binary tree
          std::copy(first, last, std::inserter(btree, btree.begin()));

          // copy the sorted elements back and overwrite
           std::copy( btree.begin(), btree.end(), first);

          return first + count;
        //      static_assert( false, "Complete the code" );
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
  inline constexpr detail::binary_sort_fn binary_sort{};

}   // namespace dte3611::sort::algorithms

#endif   // DTE3611_WEEK1_BINARY_SORT_H
