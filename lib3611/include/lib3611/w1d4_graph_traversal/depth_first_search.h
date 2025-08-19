#ifndef DTE3611_DAY4_DEPTH_FIRST_SEARCH_H
#define DTE3611_DAY4_DEPTH_FIRST_SEARCH_H

// concepts
#include "../utils/concepts/graphs.h"

#include <boost/graph/adjacency_list.hpp>

namespace dte3611::graph::algorithms
{

  template <predef::concepts::graph::BidirectionalGraph Graph_T>
  std::vector<typename Graph_T::vertex_descriptor>
  depthFirstSearch([[maybe_unused]] Graph_T const& graph,
                   [[maybe_unused]]
                   typename Graph_T::vertex_descriptor const& start)
  {
    return {};
  }

}   // namespace dte3611::graph::algorithms


#endif   // DTE3611_DAY4_DEPTH_FIRST_SEARCH_H
