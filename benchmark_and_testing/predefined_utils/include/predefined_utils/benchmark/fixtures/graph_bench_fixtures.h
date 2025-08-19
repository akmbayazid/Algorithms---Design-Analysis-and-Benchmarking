#ifndef DTE3611_PREDEF_BENCHMARKING_GRAPH_FIXTURES_H
#define DTE3611_PREDEF_BENCHMARKING_GRAPH_FIXTURES_H

// google benchmark
#include <benchmark/benchmark.h>

// boost
#include <boost/graph/adjacency_list.hpp>

// stl
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>

namespace dte3611::predef::benchmarking::graph::fixtures
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

    namespace dags
    {
      class BenchDagOne
        : public detail::GoldGraphTemplate<detail::types::BidirectionalGraph> {

        using Base
          = detail::GoldGraphTemplate<detail::types::BidirectionalGraph>;

        VD A;
        VD B;
        VD C;
        VD D;
        VD E;
        VD F;

      public:
        BenchDagOne()
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
        ~BenchDagOne() override {}


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

    }   // namespace dags

    template <typename Dag_T>
    struct BenchmarkDagTemplateF : benchmark::Fixture {

      using Dag = Dag_T;

      using benchmark::Fixture::Fixture;
      ~BenchmarkDagTemplateF() override {}

      std::unique_ptr<Dag> m_graph;

      void SetUp(const benchmark::State&) final
      {
        m_graph = std::make_unique<Dag>();
      }
      void TearDown(const benchmark::State&) final { m_graph.release(); }
    };

  }   // namespace detail

  using BenchDagOneF = detail::BenchmarkDagTemplateF<detail::dags::BenchDagOne>;

}   // namespace dte3611::predef::benchmarking::graph::fixtures

#endif   // DTE3611_PREDEF_BENCHMARKING_GRAPH_FIXTURES_H
