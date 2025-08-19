#ifndef DTE3611_WEEK2_SUBSET_SUM_H
#define DTE3611_WEEK2_SUBSET_SUM_H


// concepts
// ...

// stl
#include <vector>
#include <ranges>
#include <algorithm>
#include <set>
#include <functional>
#include <numeric>

namespace dte3611::np::algorithms
{

  namespace detail
  {

    struct subset_sum_fn {

      /************************
       *  Return type -- helper
       */
      template <typename Value_T>
      using SubsetSum_Output = std::vector<std::vector<Value_T>>;

        template <typename Value_T>
        using SubsetSum = std::vector<Value_T>;

        template <std::forward_iterator         Iterator_T,
                  std::sentinel_for<Iterator_T> Sentinel_T,
                  typename Value_T>
        void addToSum(SubsetSum_Output<Value_T>&candidate_sums, SubsetSum<Value_T> &sum, Iterator_T val, Sentinel_T end) const
        {
            //Stop criterion
            if(val == end)
                return;

            // Here is our decision problem
            // two options
            // Left: val is not included in the sum
            // recursive call to addToSum
            candidate_sums.push_back(sum);
            addToSum(candidate_sums, sum, val+1, end);

            // Right: val is included in the sum
            // recursive call to addToSum
            auto sum_incl(sum);
            sum_incl.push_back(*val);
            candidate_sums.push_back(sum_incl);
            addToSum(candidate_sums, sum_incl, val+1, end);
        }


      /**************************
       *  Iterator Range Operator
       */
      // Type Generics
      template <std::forward_iterator         Iterator_T,
                std::sentinel_for<Iterator_T> Sentinel_T,
                typename Projection_T = std::identity, typename Value_T>

      // Return value
      SubsetSum_Output<Value_T>

      // Call-operator signature
      operator()(Iterator_T begin, Sentinel_T end, Value_T target,
                 Projection_T /* projection */ = {}) const
      {

        /* function content */
          SubsetSum_Output<Value_T> candidates, output;
          auto val(begin);
          SubsetSum<Value_T> c_sum;

          //1. Call a recursion method to compare our candidate sums.

          addToSum(candidates, c_sum, val, end);

          //   addToSum(candidates, val, end).

          //2. Iterate through candidate subsets and add those with sums equal to the target sum to output.

          for_each(candidates.begin(), candidates.end(), [&](SubsetSum<Value_T>&sum){
              auto subsum = std::accumulate(sum.begin(), sum.end(), 0.);
              if(subsum == target)
                output.push_back(sum);
          });

        return output;
      }


      /******************
       *  Ranges Operator
       */

      // Type Generics
      template <std::ranges::forward_range Range_T,
                typename Projection_T = std::identity, typename Value_T>

      // Return value
      SubsetSum_Output<Value_T>

      // Call-operator signature
      operator()(Range_T&& range, Value_T target,
                 Projection_T projection = {}) const
      {
        return (*this)(std::ranges::begin(range), std::ranges::end(range),
                       target, std::ref(projection));
        //      static_assert( false, "Complete the code - find the appropriate call signature in the cpp reference documentation." );
      }
    };

  }   // namespace detail

  inline constexpr detail::subset_sum_fn subset_sum{};

}   // namespace dte3611::np::algorithms


#endif   // DTE3611_WEEK2_SUBSET_SUM_H
