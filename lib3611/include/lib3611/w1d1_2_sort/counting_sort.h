#ifndef DTE3611_WEEK1_COUNTING_SORT_H
#define DTE3611_WEEK1_COUNTING_SORT_H

// stl
#include <iterator>
#include <algorithm>
#include<vector>

namespace dte3611::sort::algorithms
{
  namespace detail
  {

    struct counting_sort_fn {

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
                 Compare_T /*comp*/ = {}, Projection_T /*proj*/ = {}) const
      {
          auto const count = std::ranges::distance(first, last);

          if (count <= 1) return last;

          auto const max_value = *std::ranges::max_element (first, last);

          using NoEleType = typename std::iterator_traits<Iterator_T>::value_type;
            // count the occurances of each element of [value]

          std::vector<NoEleType> count_container(max_value+1,0ul);

         // std::vector<int> count_container (max_value+1,0ul);


          std::for_each(first, last, [&count_container](auto const &val){count_container[val]++;
          });

          for (NoEleType val_idx = 0ul, cnt_idx =0ul; val_idx < count_container.size(); ++val_idx){
              const NoEleType no_ele =count_container[val_idx];
              if (no_ele ==0ul) continue;

              // accumulate and write back result
              for ( NoEleType ele_nr = no_ele; ele_nr != 0; --ele_nr)
                  *(first + (cnt_idx++)) = val_idx;

          }

          return first+count;
        //        static_assert(false, "Complete the code");
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
  inline constexpr detail::counting_sort_fn counting_sort{};


}   // namespace dte3611::sort::algorithms

#endif   // DTE3611_WEEK1_COUNTING_SORT_H
