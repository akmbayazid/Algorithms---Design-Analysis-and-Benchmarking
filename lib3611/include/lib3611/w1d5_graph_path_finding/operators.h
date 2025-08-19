#ifndef DTE3611_DAY5_OPERATORS_H
#define DTE3611_DAY5_OPERATORS_H

namespace dte3611::graph::operators
{

  template <typename = void>
  struct DefaultEdgeDistanceOperator {

    template <typename Graph_T>
    double operator()([[maybe_unused]]
                      typename Graph_T::edge_descriptor const& e,
                      [[maybe_unused]] Graph_T const&          graph)
    {
      //return 1.;
        return graph[e].distance;
    }
  };

}   // namespace dte3611::graph::operators

#endif   // DTE3611_DAY5_OPERATORS_H
