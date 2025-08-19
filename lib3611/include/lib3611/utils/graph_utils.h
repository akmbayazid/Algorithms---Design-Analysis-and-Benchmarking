#ifndef DTE3611_UTILS_GRAPH_UTILS_H
#define DTE3611_UTILS_GRAPH_UTILS_H

// concepts
#include "../utils/concepts/graphs.h"

// stl
#include <vector>

namespace dte3611::graph::utils
{
  namespace detail
  {
    template <dte3611::predef::concepts::graph::BidirectionalGraph Graph_T>
    void buildResultPaths(
      std::vector<std::vector<typename Graph_T::vertex_descriptor>>& paths,
      Graph_T const& graph, typename Graph_T::vertex_descriptor start,
      std::vector<typename Graph_T::vertex_descriptor> const& predecessors,
      typename Graph_T::vertex_descriptor const&              v,
      std::vector<typename Graph_T::vertex_descriptor>        a_path)
    {
      if (v == start) {
        std::reverse(std::begin(a_path), std::end(a_path));
        paths.push_back(a_path);
        return;   // register path
      }

      if (predecessors[v] == v) return;   // dead end -- no shortest path

      a_path.push_back(v);
      for (auto [ei, end] = boost::in_edges(v, graph); ei != end; ei++) {
        auto s = boost::source(*ei, graph);
        buildResultPaths(paths, graph, start, predecessors, s, a_path);
      }
    }

    //- NB! Incomplete. Currently returns only one shortest path.
    template <dte3611::predef::concepts::graph::BidirectionalGraph Graph_T>
    void buildShortestPathsFromPredecessors(
      std::vector<std::vector<typename Graph_T::vertex_descriptor>>& paths,
      Graph_T const& graph, typename Graph_T::vertex_descriptor start,
      std::vector<typename Graph_T::vertex_descriptor> const& predecessors,
      typename Graph_T::vertex_descriptor const&              v,
      std::vector<typename Graph_T::vertex_descriptor>        a_path)
    {
      if (v == start) {
        std::reverse(std::begin(a_path), std::end(a_path));
        paths.push_back(a_path);
        return;   // register path
      }

      if (predecessors[v] == v) return;   // dead end -- no shortest path

      a_path.push_back(v);
      buildShortestPathsFromPredecessors(paths, graph, start, predecessors,
                                        predecessors[v], a_path);
    }
  }   // namespace detail
}   // namespace dte3611::graph::utils

#endif
