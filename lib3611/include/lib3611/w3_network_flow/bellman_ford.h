#ifndef DTE3611_WEEK2_BELLMAN_FORD_H
#define DTE3611_WEEK2_BELLMAN_FORD_H

#include "../w1d5_graph_path_finding/operators.h"

// concepts
#include "../utils/concepts/graphs.h"
#include "../utils/concepts/operators.h"

// stl
#include <vector>
#include <map>

// utils
#include "../utils/graph_utils.h"

// Boost
#include <boost/property_map/function_property_map.hpp>

namespace dte3611::graph::algorithms
{

  template <predef::concepts::graph::BidirectionalGraph            Graph_T,
            predef::concepts::graph::EdgeCapacityOperator<Graph_T> EdOp_T
            = operators::DefaultEdgeDistanceOperator<>>
  std::vector<std::vector<typename Graph_T::vertex_descriptor>>
  bellmanFordShortestPaths([[maybe_unused]] Graph_T const& graph,
                           [[maybe_unused]]
                           typename Graph_T::vertex_descriptor const& start,
                           [[maybe_unused]]
                           typename Graph_T::vertex_descriptor const& goal,
                           [[maybe_unused]] EdOp_T distance_op = EdOp_T())
  {
    using VertexDescriptor = typename Graph_T::vertex_descriptor;
    using EdgeDescriptor   = typename Graph_T::edge_descriptor;
    using DistanceType     = double;
      std::map<VertexDescriptor, DistanceType>d;
      std::map<VertexDescriptor, VertexDescriptor>successor;

      //initialize

      for(auto [v,v_end] = boost::vertices(graph); v!= v_end; ++v){
          d.emplace(*v, std::numeric_limits<DistanceType>:: max());
          successor.emplace(*v, *v_end);
      }
      d[goal] = DistanceType{0};
      auto const n = boost::num_vertices(graph);
      std::vector<VertexDescriptor> prev_update;
      std::vector<VertexDescriptor> update_i;
      update_i.push_back(goal);

      for (auto i=1;i<n;++i){
          prev_update.swap(update_i);
          update_i.clear();
          //for each node w in V
          for (auto [v_iter,v_end]= boost::vertices(graph);v_iter!=v_end;++v_iter){
              auto const& w = *v_iter;
              //if d was uodated ib previous pass
              bool w_found_in_prev_update = std::find(prev_update.begin(), prev_update.end(),w)!= prev_update.end();
              if(w_found_in_prev_update ){




              //for each edge (v,w) in E
                  for(auto [e, e_end]= boost::in_edges(w,graph); e!= e_end;++e){
                      auto const v = boost::source(*e,graph);
                      auto const l_vw = distance_op(*e,graph);



              //if (d[v] > d[w] + l[t])
                      if(d[v] > d[w] + l_vw){
                          d[v]= d[w] + l_vw;
                          successor[v]= w;
                          update_i.push_back(v);


                      }
                   }
              }
          }
          // if no d[.] value changed in pass i , then stop
          if(update_i.empty()) break;
      }


// Now we hvae build a list of succesors
      //what if it does not contain goal?
      if (!successor.contains(goal)) return {};
      //The algorithm should return a list of(multiple) shortest paths

      using AShortestPath = std::vector<VertexDescriptor>;
      using ShortestPaths = std::vector<AShortestPath>;

      AShortestPath path;
      ShortestPaths shortest_paths;
      auto invalid = boost::vertices(graph).second;

      //build shortest path from successors
      for (auto v = successor [start];v!=successor[goal]; v = successor[v]){
          if(v == *invalid)
              return{};
          path.push_back(v);
      }
      if(!path.empty())
        shortest_paths.push_back(path);

    return shortest_paths;

 }

} // namespace

#endif
