#ifndef DTE3611_WEEK2_KNAPSACK_H
#define DTE3611_WEEK2_KNAPSACK_H

// stl
//#include <iterator>
#include <ranges>
#include <algorithm>
#include<vector>

namespace dte3611::np::algorithms
{

  namespace detail
  {

    struct knapsack_01_fn {

      /**************************
       *  Iterator Range Operator
       */

      // Type Generics
      template <std::input_iterator           Iterator_T,
                std::sentinel_for<Iterator_T> Sentinel_T,
                std::weakly_incrementable     Output_T,
                typename ValueProjection_T  = std::identity,
                typename WeightProjection_T = std::identity>
      // Return value
      int
      // Call-operator signature
      operator()(Iterator_T begin , Sentinel_T  end ,
                 Output_T  result , int  capacity ,
                 ValueProjection_T  value_projection,
                 WeightProjection_T weight_projection) const
      {
          auto const n= std::ranges::distance(begin, end);
          auto const W = capacity;

          //Invoke weight projection and determine it's type
          using w_type = typename std::decay_t<decltype(std::invoke(weight_projection, begin))>;

          using M_type = std::vector<std::vector<w_type>>;

          //Declare Memorization table

          // M of size n x W
          M_type M;


          //Remember to resize M[][] first)
          M.resize(size_t(n+1),{});
          for(auto i=M.begin();i!= M.end(); ++i)
              (*i).resize(size_t(W+1),0);

          //Now compute the values for M
          for (auto w=0; w < W; ++w)
              M[0][w] = 0;
          for (auto i=1; i<= n; ++i){
              auto const& i_item = *(begin+i - 1);
              auto const vi = std::invoke(value_projection, i_item);
              auto const wi = std::invoke(weight_projection, i_item);

              for(auto w=0; w<=W; ++w){
                  if(wi > w){
                      //does not select item i

                      M[i] [w] = M[i-1] [w];

                  }
                  else {
                      // select item i
                      // collect value vi
                      // new weight limit w - wi
                      //selects the best of {1,2,..., i-1} subject to W

                      M[i][w] = std::max(M[i-1][w], vi +M[i-1][w-wi]);
                  }
              }
          }
          // Hint: dynamic programming as outlined in the algorithm from lecture



          // Now implement back-tracking to populate the result container

          auto current = W;
          for(auto i=n; i>0; i--){
              auto const& i_item = *(begin+i - 1);
               auto const wi = std::invoke(weight_projection, i_item);
               if(M[i][current]> M[i-1][current])
               {
                   std::fill_n(result,1,true);
                   current = current-wi;
               }
               else
                    std::fill_n(result,1,false);

          }
        return M[n] [W];

      }

      /******************
       *  Ranges Operator
       */

      // Type Generics
      template <std::ranges::input_range  Range_T,
                std::weakly_incrementable Output_T,
                typename ValueProjection_T  = std::identity,
                typename WeightProjection_T = std::identity>
      // Return value
      auto
      // Call-operator signature
      operator()(Range_T&& range, Output_T result, int capacity,
                 ValueProjection_T  value_projection,
                 WeightProjection_T weight_projection) const
      {
        return (*this)(std::ranges::begin(range), std::ranges::end(range),
                       std::move(result), capacity, std::ref(value_projection),
                       std::ref(weight_projection));
        //      static_assert( false, "Complete the code - find the appropriate call signature in the cpp reference documentation." );
      }

    };

  }   // namespace detail

  inline constexpr detail::knapsack_01_fn knapsack_01{};


}   // namespace dte3611::np::algorithms


#endif   // DTE3611_WEEK2_KNAPSACK_H
