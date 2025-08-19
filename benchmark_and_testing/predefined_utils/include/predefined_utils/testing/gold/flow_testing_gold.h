#ifndef DTE3611_PREDEF_TESTING_FLOW_GOLD_H
#define DTE3611_PREDEF_TESTING_FLOW_GOLD_H

// boost
#include <boost/graph/adjacency_list.hpp>

// gtest
#include <gtest/gtest.h>

// stl
#include <concepts>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <random>

namespace dte3611::predef::testing::flow
{

  namespace detail
  {

    namespace types
    {

      namespace properties
      {

        struct NodeProperties {
          std::string name;
        };

        struct EdgeProperties {
          int flow;
          int capacity;
          int cost;
        };

        using GraphProperties = boost::no_property;

      }   // namespace properties

      using BidirectionalGraph
        = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                                properties::NodeProperties,
                                properties::EdgeProperties,
                                properties::GraphProperties>;
      using DirectedGraph
        = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                                properties::NodeProperties,
                                properties::EdgeProperties,
                                properties::GraphProperties>;

    }   // namespace types


    template <typename GraphType_T>
    struct GoldGraphTemplate {

      using Graph       = GraphType_T;
      using VP          = typename Graph::vertex_property_type;
      using EP          = typename Graph::edge_property_type;
      using VD          = typename Graph::vertex_descriptor;
      using ED          = typename Graph::edge_descriptor;
      using VDVector    = std::vector<VD>;
      using VDVecVector = std::vector<VDVector>;

      GoldGraphTemplate()          = default;
      virtual ~GoldGraphTemplate() = default;

      Graph& graph() { return m_graph; }

      Graph m_graph;
    };

  }   // namespace detail

  namespace gold
  {





    class FordFulkersonDemoDAG
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;
      VD B;
      VD C;
      VD D;
      VD S;
      VD T;

    public:
      FordFulkersonDemoDAG()
      {
        // clang-format off
      /*!
       *         A  -->  B
       *
       *     ^   |  \    ^   \
       *    /    V   v   |    v
       *
       * S  -->  C  -->  D  -->  T
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);
        B = boost::add_vertex(VP{.name = "B"}, m_graph);
        C = boost::add_vertex(VP{.name = "C"}, m_graph);
        D = boost::add_vertex(VP{.name = "D"}, m_graph);
        S = boost::add_vertex(VP{.name = "S"}, m_graph);
        T = boost::add_vertex(VP{.name = "T"}, m_graph);

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(S, A, EP{.flow = 0, .capacity = 10 }, m_graph);
        boost::add_edge(S, C, EP{.flow = 0, .capacity = 10 }, m_graph);
        boost::add_edge(A, D, EP{.flow = 0, .capacity = 8 }, m_graph);
        boost::add_edge(A, C, EP{.flow = 0, .capacity = 2 }, m_graph);
        boost::add_edge(A, B, EP{.flow = 0, .capacity = 4 }, m_graph);
        boost::add_edge(B, T, EP{.flow = 0, .capacity = 10 }, m_graph);
        boost::add_edge(C, D, EP{.flow = 0, .capacity = 9 }, m_graph);
        boost::add_edge(D, T, EP{.flow = 0, .capacity = 10 }, m_graph);
        boost::add_edge(D, B, EP{.flow = 0, .capacity = 6 }, m_graph);
      }
      ~FordFulkersonDemoDAG() override {}


      VD const& a() const { return A; }
      VD const& b() const { return B; }
      VD const& c() const { return C; }
      VD const& d() const { return D; }
      VD const& s() const { return S; }
      VD const& t() const { return T; }

//      VDVecVector shortestPathsAEGold() const { return {{B, E}}; }
//      VDVecVector shortestPathsACGold() const { return {{B, E, F, C}}; }

      int maxFlowSTGold() const { return 19; }
    };



    class FordFulkersonDemoDAG2
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;
      VD B;
      VD C;
      VD D;
      VD S;
      VD T;

    public:
      FordFulkersonDemoDAG2()
      {
        // clang-format off
      /*!
       *         A  -->  D
       *
       *     ^   ^    /  ^   \
       *    /    |  v    |    v
       *
       * S  -->  C  -->  C  -->  T
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);
        B = boost::add_vertex(VP{.name = "B"}, m_graph);
        C = boost::add_vertex(VP{.name = "C"}, m_graph);
        D = boost::add_vertex(VP{.name = "D"}, m_graph);
        S = boost::add_vertex(VP{.name = "S"}, m_graph);
        T = boost::add_vertex(VP{.name = "T"}, m_graph);

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(S, A, EP{.flow = 1, .capacity = 3 }, m_graph);
        boost::add_edge(S, B, EP{.flow = 2, .capacity = 2 }, m_graph);
        boost::add_edge(B, A, EP{.flow = 1, .capacity = 3 }, m_graph);
        boost::add_edge(A, D, EP{.flow = 2, .capacity = 2 }, m_graph);
        boost::add_edge(D, B, EP{.flow = 1, .capacity = 1 }, m_graph);
        boost::add_edge(B, C, EP{.flow = 2, .capacity = 3 }, m_graph);
        boost::add_edge(C, D, EP{.flow = 1, .capacity = 3 }, m_graph);
        boost::add_edge(D, T, EP{.flow = 2, .capacity = 3 }, m_graph);
        boost::add_edge(C, T, EP{.flow = 1, .capacity = 2 }, m_graph);
      }
      ~FordFulkersonDemoDAG2() override {}


      VD const& a() const { return A; }
      VD const& b() const { return B; }
      VD const& c() const { return C; }
      VD const& d() const { return D; }
      VD const& s() const { return S; }
      VD const& t() const { return T; }

//      VDVecVector shortestPathsAEGold() const { return {{B, E}}; }
//      VDVecVector shortestPathsACGold() const { return {{B, E, F, C}}; }

      int maxFlowSTGold() const { return 4; }
    };


    class MinCostMaxFlowDAG
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;
      VD B;
      VD C;
      VD D;
      VD E;
      VD F;
      VD S;
      VD T;

    public:
      MinCostMaxFlowDAG()
      {
        // clang-format off
      /*!
       *         A  -->  D
       *
       *     ^   |  \    |   \
       *    /    v    v  v    v
       *
       * S  -->  B  -->  E  -->  T
       *
       *    \    |  \    |    ^
       *     v   v   v   v   /
       *
       *         C  -->  F
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);
        B = boost::add_vertex(VP{.name = "B"}, m_graph);
        C = boost::add_vertex(VP{.name = "C"}, m_graph);
        D = boost::add_vertex(VP{.name = "D"}, m_graph);
        E = boost::add_vertex(VP{.name = "E"}, m_graph);
        F = boost::add_vertex(VP{.name = "F"}, m_graph);
        S = boost::add_vertex(VP{.name = "S"}, m_graph);
        T = boost::add_vertex(VP{.name = "T"}, m_graph);

        // bgl has "implicit" alphanumeric child-ordering

        // 3 extra nodes to allow for a max flow solver in a related network
        // Supply
        boost::add_edge(S, A, EP{.flow = 0, .capacity = 12 }, m_graph);
        boost::add_edge(S, B, EP{.flow = 0, .capacity = 6 }, m_graph);
        boost::add_edge(S, C, EP{.flow = 0, .capacity = 14}, m_graph);

        boost::add_edge(A, B, EP{.flow = 0, .capacity = 11, .cost = 4},
                        m_graph);
        boost::add_edge(A, D, EP{.flow = 0, .capacity = 5, .cost = 5}, m_graph);
        boost::add_edge(A, E, EP{.flow = 0, .capacity = 2, .cost = 12},
                        m_graph);

        boost::add_edge(B, C, EP{.flow = 0, .capacity = 9, .cost = 6}, m_graph);
        boost::add_edge(B, E, EP{.flow = 0, .capacity = 4, .cost = 4}, m_graph);
        boost::add_edge(B, F, EP{.flow = 0, .capacity = 2, .cost = 6}, m_graph);

        boost::add_edge(C, F, EP{.flow = 0, .capacity = 31, .cost = 3},
                        m_graph);

        boost::add_edge(D, E, EP{.flow = 0, .capacity = 18, .cost = 4},
                        m_graph);

        boost::add_edge(E, F, EP{.flow = 0, .capacity = 9, .cost = 5}, m_graph);

        // 3 extra nodes to allow for a max flow solver in a related network
        // Demand
        boost::add_edge(E, T, EP{.flow = 0, .capacity = 7 }, m_graph);
        boost::add_edge(D, T, EP{.flow = 0, .capacity = 3 }, m_graph);
        boost::add_edge(F, T, EP{.flow = 0, .capacity = 22 }, m_graph);
      }
      ~MinCostMaxFlowDAG() override {}


      VD const& a() const { return A; }
      VD const& b() const { return B; }
      VD const& c() const { return C; }
      VD const& d() const { return D; }
      VD const& e() const { return E; }
      VD const& f() const { return F; }
      VD const& s() const { return S; }
      VD const& t() const { return T; }

//      VDVecVector shortestPathsAEGold() const { return {{B, E}}; }
//      VDVecVector shortestPathsACGold() const { return {{B, E, F, C}}; }

      int maxFlowSTGold() const { return 32; }
    };


  }   // namespace gold

}   // namespace dte3611::predef::testing::flow

#endif   // DTE3611_PREDEF_TESTING_FLOW_GOLD_H
