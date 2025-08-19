#ifndef DTE3611_WEEK3_NETWORK_FLOW_H
#define DTE3611_WEEK3_NETWORK_FLOW_H
// concepts
#include "../utils/concepts/graphs.h"
#include "../utils/concepts/operators.h"
// stl
#include <vector>
// boost
#include <boost/graph/edge_list.hpp>
#include "operators.h"
namespace dte3611::np::algorithms
{
  template <predef::concepts::graph::BidirectionalGraph Graph_T,
            predef::concepts::graph::EdgeCapacityOperator<Graph_T> EdCOp_T =
                graph::operators::DefaultEdgeCapacityOperator<>,
            predef::concepts::graph::EdgeCapacityOperator<Graph_T> EdFOp_T =
                graph::operators::DefaultEdgeFlowOperator<>>
  int maxFlow([[maybe_unused]] Graph_T& graph,
              [[maybe_unused]] typename Graph_T::vertex_descriptor const& source,
              [[maybe_unused]] typename Graph_T::vertex_descriptor const& sink,
              [[maybe_unused]] EdCOp_T capacity_op = EdCOp_T(),
              [[maybe_unused]] EdFOp_T flow_op = EdFOp_T()) {
    // Implement Ford-Fulkerson (method) for computing the maximum flow in a flow
    // network by augmenting path.
    // Note: Edmonds-Karp is a fully defined implementation of the Ford-Fulkerson
    // method.
    //- Initialize flows for all edges to 0
    for(auto [e,e_end] = boost::edges(graph); e != e_end; ++e){
        auto f = flow_op(*e, graph);
        f=0;
    }
    //- Construct residual graph
    Graph_T r_graph;
    using EP = typename Graph_T::edge_property_type;
    using VP = typename Graph_T::vertex_property_type;
    for (auto [v, v_end] = boost::vertices(graph); v!= v_end; ++v)
    {
        //copy vertices from graph to r_graph
        boost::add_vertex(VP{.name = graph[*v].name}, r_graph);
    }
    for(auto [e,e_end] = boost::edges(graph); e != e_end; ++e){
        auto u = boost::source(*e, graph);
        auto v = boost::target(*e, graph);
        auto capacity = graph[*e].capacity;
        auto flow = graph[*e].flow;
    //forward edge
       boost::add_edge(u,v, EP{.flow = 0, .capacity = capacity - flow}, r_graph);
    //reverse edge
       boost::add_edge(v,u, EP{.flow = 0, .capacity = flow}, r_graph);
    }
    // Bfs implementation
    using VertexDescriptor = typename Graph_T::vertex_descriptor;
    using VertexDescriptors = std::map<VertexDescriptor, VertexDescriptor>;
    auto bfs = [](VertexDescriptors &predecessors,
                  VertexDescriptor const &start,
                  VertexDescriptor const &goal,
                  Graph_T const &graph,
                  graph::operators::template ResidualCapacityOperator<> residual_op) {
        predecessors.clear();
        std::vector<VertexDescriptor> visited;
        std::list<VertexDescriptor> node_queue;
        visited.push_back(start);
        node_queue.push_back(start);
        while(!node_queue.empty()) {
            auto u = node_queue.front();
            node_queue.pop_front();
            auto [adj, e_adj] = boost::adjacent_vertices(u,graph);
            for(auto i = adj; i != e_adj; ++i) {
                auto const &vi = (*i);
                auto [e, exists] = boost::edge(u, vi, graph);
                if (std::find(visited.begin(), visited.end(), vi) == visited.end()) {
                    auto res_cap = residual_op(e, graph);
                    if(exists && res_cap > 0) {
                        predecessors[vi] = u;
                        if (vi==goal)
                            return true;
                        visited.push_back(vi);
                        node_queue.push_back(vi);
                    }
                }
            }
        }
        predecessors.clear();
        return false;
    };
    // at first we declare bottleneck capacity function with a lambda for easy try outs
    auto bottleneck_capacity = [](auto const& P, auto const& G) {
        // We search the min capacity, so we initiate with big number
        auto delta =std::numeric_limits<int>::max();
        // We now need to traverse the edges
        auto u = *P.begin();
        for (auto i = ++P.begin(); i != P.end(); ++i) {
            auto v = *i;
            //std::pair<edge_descriptor, bool> edge = boost::edge(u, v, G)
            auto [e, exists] = boost::edge(u, v, G);
            if(exists) {
                delta = std::min(delta, graph::operators::ResidualCapacityOperator<>()(e, G));
            }
            u = v;
        }
        return delta;
    };
    // We update also the residual graph as we go
    auto augment = [&bottleneck_capacity](auto const &P,
                                          auto &graph,
                                          auto &r_graph) {
        //Grab bottleneck capacity using the function we defined
        auto delta = bottleneck_capacity(P, r_graph);
        auto u = *P.begin();
        for (auto i = ++P.begin(); i!= P.end(); ++i) {
            auto v = *i;
            auto [e, exists] =  boost::edge(u, v, graph);
            // erev is e^reverse (on a trouvé un path dans le residual graph qui était le long
            // d'un reverse, donc on doit prendre le edge dans l'autre sens et retirer du flow
            // dans le graph originel)
            auto [erev, erevexists] = boost::edge(v, u, graph);
            auto [re, rexists] = boost::edge(u, v, r_graph);
            auto [arc, aexists] = boost::edge(v, u, r_graph);
            if (exists) {
                graph[e].flow += delta;
            }
            else {
                graph[erev].flow -= delta;
            }
            // Update residual capacities
            r_graph[re].capacity -= delta;
            r_graph[arc].capacity += delta;
            u = v;
        }
    };
    // Created to add up the total flow
    // (2 ways of doing it: either adding all outgoing flow form s, or all ingoing flow to t)
    auto f = [&](auto const &source, auto const &graph) {
        auto [e, e_end] = boost::out_edges(source, graph);
        int flow = 0;
        for (auto i = e; i!= e_end; ++i) {
            flow += flow_op(*i, graph);
        }
        return flow;
    };
    //Clear container
    VertexDescriptors predecessors;
    while(bfs(predecessors, source, sink, r_graph, graph::operators::ResidualCapacityOperator<>())) {
        //augment
        std::vector<VertexDescriptor> node_path;
        //populate node path based on sink, predecessors, source
        node_path.push_back(sink);
        auto pre = predecessors.at(sink);
        while(pre != source) {
            node_path.push_back(pre);
            pre = predecessors.at(pre);
        }
        node_path.push_back(source);
        std::reverse(node_path.begin(), node_path.end());
        //augment node path
        augment(node_path, graph, r_graph);
    }
    //Should return flow instead !
    return f(source, graph);
  }
  template <typename Graph_T>
  void cycleCanceling([[maybe_unused]] Graph_T& graph)
  {
    // 1. Find a feasible flow f   (Ford-Fulkerson maxflow in a related graph)
    // 2. REPEAT (until no augmenting cycles)
    //   2.1 Find an augmenting cycle C (Bellman-Ford in residual graph to detect negative cost cycles)
    //   2.2 Augment flow along C
  }
}   // namespace dte3611::graph::algorithms
#endif   // DTE3611_WEEK3_NETWORK_FLOW_H
