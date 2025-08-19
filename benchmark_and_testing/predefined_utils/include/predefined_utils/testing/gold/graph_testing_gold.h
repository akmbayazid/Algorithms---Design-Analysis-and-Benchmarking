#ifndef DTE3611_PREDEF_TESTING_GRAPH_FIXTURES_H
#define DTE3611_PREDEF_TESTING_GRAPH_FIXTURES_H

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

namespace dte3611::predef::testing::graph
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
          double distance;
        };

        using GraphProperties = boost::no_property;

      }   // namespace properties

      using BidirectionalGraph
        = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
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

      Graph const& graph() const { return m_graph; }

      Graph m_graph;
    };

  }   // namespace detail

  namespace gold
  {
    class SingleNodeDAG
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;

    public:
      SingleNodeDAG()
      {
        // clang-format off
      /*!
       *   A
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);
      }
      ~SingleNodeDAG() override {}

      VD const& a() const { return A; }

      VDVector gold() const { return {}; }
    };


    class TreeDAG
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;
      VD B;
      VD C;
      VD D;
      VD E;

    public:
      TreeDAG()
      {
        // clang-format off
      /*!
       *   A  -->  B  -->  C --> D
       *
       *           |
       *           V
       *
       *           E
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);
        B = boost::add_vertex(VP{.name = "B"}, m_graph);
        C = boost::add_vertex(VP{.name = "C"}, m_graph);
        D = boost::add_vertex(VP{.name = "D"}, m_graph);
        E = boost::add_vertex(VP{.name = "E"}, m_graph);

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(A, B, EP{.distance = 1.}, m_graph);
        boost::add_edge(B, C, EP{.distance = 1.}, m_graph);
        boost::add_edge(B, E, EP{.distance = 1.}, m_graph);
        boost::add_edge(C, D, EP{.distance = 1.}, m_graph);
      }
      ~TreeDAG() override {}

      VD const& a() const { return A; }
      VD const& b() const { return B; }
      VD const& c() const { return C; }
      VD const& d() const { return D; }


      VDVector dfsFromAGold() const { return {B, C, D, E}; }
      VDVector bfsFromAGold() const { return {B, C, E, D}; }
    };





    class DAGOne
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;
      VD B;
      VD C;
      VD D;
      VD E;
      VD F;

    public:
      DAGOne()
      {
        // clang-format off
      /*!
       *   A  -->  B       C
       *
       *   ^       |    ^  ^
       *   |       V   /   |
       *
       *   D  -->  E  -->  F
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);   // 0
        B = boost::add_vertex(VP{.name = "B"}, m_graph);   // 1
        C = boost::add_vertex(VP{.name = "C"}, m_graph);   // 2
        D = boost::add_vertex(VP{.name = "D"}, m_graph);   // 3
        E = boost::add_vertex(VP{.name = "E"}, m_graph);   // 4
        F = boost::add_vertex(VP{.name = "F"}, m_graph);   // 5

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(A, B, EP{.distance = 1.}, m_graph);
        boost::add_edge(B, E, EP{.distance = 1.}, m_graph);
        boost::add_edge(D, A, EP{.distance = 1.}, m_graph);
        boost::add_edge(D, E, EP{.distance = 1.}, m_graph);
        boost::add_edge(E, C, EP{.distance = 10.}, m_graph);
        boost::add_edge(E, F, EP{.distance = 1.}, m_graph);
        boost::add_edge(F, C, EP{.distance = 1.}, m_graph);
      }
      ~DAGOne() override {}


      VD const& a() const { return A; }
      VD const& b() const { return B; }
      VD const& c() const { return C; }
      VD const& d() const { return D; }
      VD const& e() const { return E; }
      VD const& f() const { return F; }

      VDVector dfsFromAGold() const { return {B, E, C, F}; }
      VDVector dfsFromBGold() const { return {E, C, F}; }
      VDVector dfsFromCGold() const { return {}; }
      VDVector dfsFromDGold() const { return {A, B, E, C, F}; }
      VDVector dfsFromEGold() const { return {C, F}; }
      VDVector dfsFromFGold() const { return {C}; }

      VDVector bfsFromAGold() const { return {B, E, C, F}; }
      VDVector bfsFromBGold() const { return {E, C, F}; }
      VDVector bfsFromCGold() const { return {}; }
      VDVector bfsFromDGold() const { return {A, E, B, C, F}; }
      VDVector bfsFromEGold() const { return {C, F}; }
      VDVector bfsFromFGold() const { return {C}; }

      VDVecVector shortestPathsAEGold() const { return {{B, E}}; }
      VDVecVector shortestPathsACGold() const { return {{B, E, F, C}}; }
    };



    class DAGTwo
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;
      VD B;
      VD C;
      VD D;
      VD E;
      VD F;
      VD G;
      VD H;

    public:
      DAGTwo()
      {
        // clang-format off
      /*!
       *           H  -->  G
       *
       *           ^       ^
       *           |       |
       *
       *   A  -->  B       C
       *
       *   ^       ^    ^  |
       *   |       |   /   v
       *
       *   D  -->  E  -->  F
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);   // 0
        B = boost::add_vertex(VP{.name = "B"}, m_graph);   // 1
        C = boost::add_vertex(VP{.name = "C"}, m_graph);   // 2
        D = boost::add_vertex(VP{.name = "D"}, m_graph);   // 3
        E = boost::add_vertex(VP{.name = "E"}, m_graph);   // 4
        F = boost::add_vertex(VP{.name = "F"}, m_graph);   // 5
        G = boost::add_vertex(VP{.name = "G"}, m_graph);   // 6
        H = boost::add_vertex(VP{.name = "H"}, m_graph);   // 7

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(A, B, EP{.distance = 1.}, m_graph);
        boost::add_edge(B, H, EP{.distance = 1.}, m_graph);
        boost::add_edge(C, F, EP{.distance = 1.}, m_graph);
        boost::add_edge(C, G, EP{.distance = 1.}, m_graph);
        boost::add_edge(D, A, EP{.distance = 1.}, m_graph);
        boost::add_edge(D, E, EP{.distance = 1.}, m_graph);
        boost::add_edge(E, B, EP{.distance = 1.}, m_graph);
        boost::add_edge(E, C, EP{.distance = 1.}, m_graph);
        boost::add_edge(E, F, EP{.distance = 1.}, m_graph);
        boost::add_edge(G, H, EP{.distance = 1.}, m_graph);
      }
      ~DAGTwo() override {}


      VD const& a() const { return A; }
      VD const& b() const { return B; }
      VD const& c() const { return C; }
      VD const& d() const { return D; }
      VD const& e() const { return E; }
      VD const& f() const { return F; }
      VD const& g() const { return G; }
      VD const& h() const { return H; }

      VDVector dfsFromAGold() const { return {B, H}; }
      VDVector dfsFromBGold() const { return {H}; }
      VDVector dfsFromCGold() const { return {F, G, H}; }
      VDVector dfsFromDGold() const { return {A, B, H, E, C, F, G}; }
      VDVector dfsFromEGold() const { return {B, H, C, F, G}; }
      VDVector dfsFromFGold() const { return {}; }
      VDVector dfsFromGGold() const { return {H}; }
      VDVector dfsFromHGold() const { return {}; }

      VDVector bfsFromAGold() const { return {B, H}; }
      VDVector bfsFromBGold() const { return {H}; }
      VDVector bfsFromCGold() const { return {F, G, H}; }
      VDVector bfsFromDGold() const { return {A, E, B, C, F, H, G}; }
      VDVector bfsFromEGold() const { return {B, C, F, H, G}; }
      VDVector bfsFromFGold() const { return {}; }
      VDVector bfsFromGGold() const { return {H}; }
      VDVector bfsFromHGold() const { return {}; }

      VDVecVector shortestPathsAEGold() const { return {}; }
      VDVecVector shortestPathsACGold() const { return {}; }
    };



    class DAGThree
      : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD A;
      VD B;
      VD C;
      VD D;
      VD E;
      VD F;

    public:
      DAGThree()
      {
        // clang-format off
      /*!
       *   A  -->  B  -->  C
       *
       *   |       |    ^  ^
       *   V       V   /   |
       *
       *   D  -->  E  -->  F
       */
        // clang-format on

        A = boost::add_vertex(VP{.name = "A"}, m_graph);   // 0
        B = boost::add_vertex(VP{.name = "B"}, m_graph);   // 1
        C = boost::add_vertex(VP{.name = "C"}, m_graph);   // 2
        D = boost::add_vertex(VP{.name = "D"}, m_graph);   // 3
        E = boost::add_vertex(VP{.name = "E"}, m_graph);   // 4
        F = boost::add_vertex(VP{.name = "F"}, m_graph);   // 5

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(A, B, EP{.distance = 1.}, m_graph);
        boost::add_edge(A, D, EP{.distance = 2.}, m_graph);
        boost::add_edge(B, C, EP{.distance = 4.}, m_graph);
        boost::add_edge(B, E, EP{.distance = 2.}, m_graph);
        boost::add_edge(D, E, EP{.distance = 1.}, m_graph);
        boost::add_edge(E, C, EP{.distance = 2.}, m_graph);
        boost::add_edge(E, F, EP{.distance = 1.}, m_graph);
        boost::add_edge(F, C, EP{.distance = 1.}, m_graph);
      }
      ~DAGThree() override {}


      VD const& a() const { return A; }
      VD const& b() const { return B; }
      VD const& c() const { return C; }
      VD const& d() const { return D; }
      VD const& e() const { return E; }
      VD const& f() const { return F; }

      VDVector dfsFromAGold() const { return {B, C, E, F, D}; }
      VDVector dfsFromBGold() const { return {C, E, F}; }
      VDVector dfsFromCGold() const { return {}; }
      VDVector dfsFromDGold() const { return {E, C, F}; }
      VDVector dfsFromEGold() const { return {C, F}; }
      VDVector dfsFromFGold() const { return {C}; }

      VDVector bfsFromAGold() const { return {B, D, C, E, F}; }
      VDVector bfsFromBGold() const { return {C, E, F}; }
      VDVector bfsFromCGold() const { return {}; }
      VDVector bfsFromDGold() const { return {E, C, F}; }
      VDVector bfsFromEGold() const { return {C, F}; }
      VDVector bfsFromFGold() const { return {C}; }

      VDVecVector shortestPathsAEGold() const { return {{B, E}, {D, E}}; }
      VDVecVector shortestPathsACGold() const
      {
        return {{B, C}, {B, E, C}, {B, E, F, C}, {D, E, C}, {D, E, F, C}};
      }
    };

    class DAG_Neg_One
        : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {
      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD S;
      VD T;
      VD V;
      VD W;

     public:
      DAG_Neg_One() {
        // clang-format off
          /*!
           *   S  -->  V
           *
           *   |  \    |
           *   V   \   V
           *        >
           *   T  <--  W
           */
        // clang-format on

        S = boost::add_vertex(VP{.name = "S"}, m_graph);  // 0
        T = boost::add_vertex(VP{.name = "T"}, m_graph);  // 1
        V = boost::add_vertex(VP{.name = "V"}, m_graph);  // 2
        W = boost::add_vertex(VP{.name = "W"}, m_graph);  // 3

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(S, T, EP{.distance = 2.}, m_graph);
        boost::add_edge(S, V, EP{.distance = 6.}, m_graph);
        boost::add_edge(S, W, EP{.distance = 4.}, m_graph);
        boost::add_edge(V, W, EP{.distance = -8.}, m_graph);
        boost::add_edge(W, T, EP{.distance = 3.}, m_graph);
      }
      ~DAG_Neg_One() override {}

      VD const& s() const { return S; }
      VD const& t() const { return T; }
      VD const& v() const { return V; }
      VD const& w() const { return W; }

      VDVecVector shortestPathsSTGold() const { return {{V, W, T}}; }
    };

    class DAG_Neg_Two
        : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {
      using Base = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

      VD S;
      VD T;
      VD A;
      VD B;
      VD C;
      VD D;
      VD E;
      VD F;

     public:
      DAG_Neg_Two() {
        // clang-format off
          /*!
           *
           * Graph from 06DPII p.32
           *
           *           A         -->       B
           *       ^
           *      /    |               ^
           *           v    \         /
           *   S             v
           *      \
           *        >  C   ->   D            \
           *   |              ^               v
           *   v   ^     \   /
           *      /       v          \
           *              E            v
           *        >          \
           *       /             >
           *   F         ->                     T
           *
           *
           */
        // clang-format on

        S = boost::add_vertex(VP{.name = "S"}, m_graph);  // 0
        T = boost::add_vertex(VP{.name = "T"}, m_graph);  // 1
        A = boost::add_vertex(VP{.name = "A"}, m_graph);  // 2
        B = boost::add_vertex(VP{.name = "B"}, m_graph);  // 3
        C = boost::add_vertex(VP{.name = "C"}, m_graph);  // 2
        D = boost::add_vertex(VP{.name = "D"}, m_graph);  // 3
        E = boost::add_vertex(VP{.name = "E"}, m_graph);  // 2
        F = boost::add_vertex(VP{.name = "F"}, m_graph);  // 3

        // bgl has "implicit" alphanumeric child-ordering
        boost::add_edge(S, A, EP{.distance = 5.}, m_graph);
        boost::add_edge(S, C, EP{.distance = 8.}, m_graph);
        boost::add_edge(S, F, EP{.distance = 9.}, m_graph);
        boost::add_edge(A, B, EP{.distance = -2.}, m_graph);
        boost::add_edge(A, C, EP{.distance = 4.}, m_graph);
        boost::add_edge(A, D, EP{.distance = 12.}, m_graph);
        boost::add_edge(B, T, EP{.distance = 17.}, m_graph);
        boost::add_edge(C, D, EP{.distance = 7.}, m_graph);
        boost::add_edge(C, E, EP{.distance = -1.}, m_graph);
        boost::add_edge(D, T, EP{.distance = 11.}, m_graph);
        boost::add_edge(E, D, EP{.distance = -6.}, m_graph);
        boost::add_edge(E, T, EP{.distance = 13.}, m_graph);
        boost::add_edge(F, C, EP{.distance = 5.}, m_graph);
        boost::add_edge(F, E, EP{.distance = -3.}, m_graph);
        boost::add_edge(F, T, EP{.distance = 10.}, m_graph);
      }
      ~DAG_Neg_Two() override {}

      VD const& s() const { return S; }
      VD const& t() const { return T; }

      VDVecVector shortestPathsSTGold() const { return {{F, E, D, T}}; }
    };

  }   // namespace gold

}   // namespace dte3611::predef::testing::graph

#endif   // DTE3611_PREDEF_TESTING_GRAPH_FIXTURES_H
