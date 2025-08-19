#ifndef DTE3611_PREDEF_CONCEPTS_GRAPHS_H
#define DTE3611_PREDEF_CONCEPTS_GRAPHS_H

/*///////////////////////////////
 * Boost Graph Library Compatible
 * Concept Types
 *
 * Special Concepts for DTE 3611
 *
 * Graph Concepts
 */

// stl
#include <concepts>

// boost
#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/graph_selectors.hpp>

namespace dte3611::predef::concepts::graph
{

  template <typename Graph_T>
  concept Graph
    = requires(Graph_T                             g,   // G type existance
               typename Graph_T::vertex_descriptor v,   // VD type existance
               typename Graph_T::edge_descriptor   e    // ED type existance
    )
  {
    // Boost graph library type checks -- from Boost graph concept documentation
    requires std::default_initializable<typename Graph_T::vertex_descriptor>;
    requires std::equality_comparable<typename Graph_T::vertex_descriptor>;
    requires std::assignable_from<typename Graph_T::vertex_descriptor&,
                                  typename Graph_T::vertex_descriptor&>;

    requires std::default_initializable<typename Graph_T::edge_descriptor>;
    requires std::equality_comparable<typename Graph_T::edge_descriptor>;
    requires std::assignable_from<typename Graph_T::edge_descriptor&,
                                  typename Graph_T::edge_descriptor&>;

    // clang-format off

    // Vertex and edge descriptor access
    {boost::vertices(g)};
    {boost::edges(g)};

    // Vertex from edge descriptor access
    {boost::source(e, g)} -> std::same_as<typename Graph_T::vertex_descriptor>;
    {boost::target(e, g)} -> std::same_as<typename Graph_T::vertex_descriptor>;

    // Vertex, edge and graph property access from [*] descriptor
    // Bundled property access
    {g[v]} -> std::convertible_to<typename Graph_T::vertex_bundled>;
    {g[e]} -> std::convertible_to<typename Graph_T::edge_bundled>;
    {g[boost::graph_bundle]} -> std::convertible_to<typename Graph_T::graph_bundled>;

    // clang-format on
  };

  template <typename Graph_T>
  concept BidirectionalGraph = Graph<Graph_T>and
    std::same_as<typename Graph_T::directed_selector, boost::bidirectionalS>;


}   // namespace dte3611::predef::concepts::graph

#endif   // DTE3611_PREDEF_CONCEPTS_GRAPHS_H
