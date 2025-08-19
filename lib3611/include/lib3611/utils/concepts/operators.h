#ifndef DTE3611_PREDEF_CONCEPTS_OPERATORS_H
#define DTE3611_PREDEF_CONCEPTS_OPERATORS_H

/*///////////////////////////////
 * Boost Graph Library Compatible
 * Concept Types
 *
 * Special Concepts for DTE 3611
 *
 * Graph Operator Concepts
 */

// stl
#include <concepts>

namespace dte3611::predef::concepts::graph
{

  template <typename Op_T, typename Graph_T>
  concept EdgeDistanceOperator
    = requires(Op_T                              op,      // O type existance
               Graph_T const&                    graph,   // G type existance
               typename Graph_T::edge_descriptor e        // ED type existance
    )
  {
    // clang-format off
    {op(e, graph)}  // Distance operator existance
    -> std::floating_point;   // Distance return type
    // clang-format on
  };

  template <typename Op_T, typename Graph_T>
  concept EdgeCapacityOperator =
      requires(Op_T op,                             // O type existance
               Graph_T const& graph,                // G type existance
               typename Graph_T::edge_descriptor e  // ED type existance
      ) {
    // clang-format off
 //    {op(e, graph)}  // Capacity operator existance
 //    -> std::convertible_to<decltype(op(e, graph)), std::floating_point>;   // Capacity return type
       requires std::convertible_to<decltype(op(e, graph)), float>;
    // clang-format on
  };

  template <typename Op_T, typename Graph_T>
  concept ShortestPathHeuristicOperator =
      requires(Op_T op,                               // O type existance
               Graph_T const& graph,                  // G type existance
               typename Graph_T::vertex_descriptor v  // VD type existance
      ) {
    // clang-format off
    {op(v, v, graph)}   // 'Heuristic of two vertices' operator existance
    -> std::floating_point;   // Heuristic return type
    // clang-format on
  };

}   // namespace dte3611::predef::concepts::graph


#endif   // DTE3611_PREDEF_CONCEPTS_OPERATORS_H
