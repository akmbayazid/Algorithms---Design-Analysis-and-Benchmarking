#ifndef DTE3611_W3_OPERATORS_H
#define DTE3611_W3_OPERATORS_H

namespace dte3611::graph::operators
{

  template <typename = void>
  struct DefaultEdgeFlowOperator {

    template <typename Graph_T>
    int operator()([[maybe_unused]]
                      typename Graph_T::edge_descriptor const& e,
                      [[maybe_unused]] Graph_T const&          graph)
    {
      return graph[e].flow;
    }
  };

  template <typename = void>
  struct DefaultEdgeCapacityOperator {

    template <typename Graph_T>
    int operator()([[maybe_unused]] typename Graph_T::edge_descriptor const& e,
                   [[maybe_unused]] Graph_T const& graph)
    {
      return graph[e].capacity;
    }
  };

  template <typename = void>
  struct ResidualCapacityOperator {

    template <typename Graph_T>
    int operator()([[maybe_unused]] typename Graph_T::edge_descriptor const& e,
                   [[maybe_unused]] Graph_T const& graph)
    {

      auto const& edge = graph[e];
      auto const  r    = edge.capacity - edge.flow;
      return r;


    }
  };

}   // namespace dte3611::graph::operators

#endif   // DTE3611_W3_OPERATORS_H
