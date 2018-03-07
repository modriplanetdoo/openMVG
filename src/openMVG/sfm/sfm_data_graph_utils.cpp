// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015, 2017 WhuAegeanSea, Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_graph_utils.hpp"

#include "openMVG/graph/connectedComponent.hpp"
#include "openMVG/graph/graph.hpp"
#include "openMVG/graph/graph_builder.hpp"
#include "openMVG/types.hpp"

#include "lemon/adaptors.h"
#include "lemon/dfs.h"
#include "lemon/kruskal.h"
#include "lemon/list_graph.h"
#include "lemon/path.h"


namespace openMVG {
namespace sfm {

static void KeepOnlyReferencedElement
(
  const std::set<IndexT> & set_remainingIds,
  const matching::PairWiseMatches & matches_in,
  matching::PairWiseMatches & matches_out
)
{
  matching::PairWiseMatches kept_matches;
  for (const auto & iter_pairwise_matches : matches_in)
  {
    if (set_remainingIds.count(iter_pairwise_matches.first.first) &&
        set_remainingIds.count(iter_pairwise_matches.first.second))
    {
      kept_matches.insert(iter_pairwise_matches);
    }
  }
  matches_out.swap(kept_matches);
}

bool PairsToConnectedComponents
(
  const Pair_Set & pairs,
  bool is_biedge,
  int min_nodes,
  std::map<IndexT, std::set<IndexT>>& subgraphs_ids
)
{
  subgraphs_ids.clear();

  using Graph = graph::indexedGraph::GraphT;
  graph::indexedGraph putativeGraph(pairs);

  // For global SFM, firstly remove the not bi-edge element
  if (is_biedge)
  {
    using EdgeMapAlias = Graph::EdgeMap<bool>;
    EdgeMapAlias cutMap(putativeGraph.g);
    if (lemon::biEdgeConnectedCutEdges(putativeGraph.g, cutMap) > 0)
    {
      // Some edges must be removed because they don't follow the biEdge condition.
      using EdgeIterator = Graph::EdgeIt;
      EdgeIterator itEdge(putativeGraph.g);
      for (EdgeMapAlias::MapIt it(cutMap); it != lemon::INVALID; ++it, ++itEdge)
      {
        if (*it)
        {
          putativeGraph.g.erase(itEdge);  // remove the not bi-edge element
        }
      }
    }
  }

  // Compute all subgraphs in the putative graph
  const int connectedComponentCount = lemon::countConnectedComponents(putativeGraph.g);
  if (connectedComponentCount >= 1)
  {
    const auto map_subgraphs = graph::exportGraphToMapSubgraphs<Graph, IndexT>(putativeGraph.g);
    for (const auto & iter_map_subgraphs : map_subgraphs)
    {
      if (iter_map_subgraphs.second.size() > min_nodes)
      {
        std::set<IndexT> subgraphNodes;
        const std::set<lemon::ListGraph::Node> & ccSet = iter_map_subgraphs.second;
        for (const auto & iter2 : ccSet)
        {
          const IndexT Id = (*putativeGraph.node_map_id)[iter2];
          subgraphNodes.insert(Id);
        }
        subgraphs_ids.emplace(iter_map_subgraphs.first, subgraphNodes);
      }
    }
  }
  return !subgraphs_ids.empty();
}

bool SplitMatchesIntoSubgraphMatches
(
  const Pair_Set & pairs,
  const matching::PairWiseMatches & matches,
  bool is_biedge,
  int min_nodes,
  std::vector<matching::PairWiseMatches> & subgraphs_matches
)
{
  if (pairs.size() == 0 || matches.size() == 0)
  {
    return false;
  }
  subgraphs_matches.clear();
  std::map<IndexT, std::set<IndexT>> subgraphs_ids;
  if (PairsToConnectedComponents(pairs, is_biedge, min_nodes, subgraphs_ids))
  {
    for (const auto & iter_subgraphs_ids : subgraphs_ids)
    {
      matching::PairWiseMatches component_matches;
      KeepOnlyReferencedElement(iter_subgraphs_ids.second, matches, component_matches);
      subgraphs_matches.emplace_back(component_matches);
    }
  }
  return !subgraphs_matches.empty();
}

bool PairsToMST
(
  const Pair_Set & pairs,
  const matching::PairWiseMatches & pair_matches,
  std::map<IndexT, Pair_Set> & msts
)
{
  std::map<IndexT, std::set<IndexT>> connected_components;
  PairsToConnectedComponents(pairs, false, 2, connected_components);

  for (const auto &connected_component : connected_components)
  {
    lemon::ListGraph graph;
    lemon::ListGraph::EdgeMap<int> edge_map(graph); // weights
    std::map<IndexT, lemon::ListGraph::Node> map_index_to_node; // map view id to node id
    std::map<lemon::ListGraph::Node, IndexT> map_node_to_index; // map node id to view id

    for (const IndexT idx : connected_component.second)
    {
      lemon::ListGraph::Node node = graph.addNode();
      map_index_to_node[ idx ] = node;
      map_node_to_index[ node ] = idx;
    }

    for (const auto &pair_matches_it : pair_matches)
    {
      const Pair & pair = pair_matches_it.first;
      const matching::IndMatches & matches = pair_matches_it.second;

      // skip if either is not part of this connected component
      if ( connected_component.second.count( pair.first ) == 0 || connected_component.second.count( pair.second ) == 0 )
        continue;

      // add edge to the graph
      lemon::ListGraph::Edge edge = graph.addEdge(map_index_to_node[ pair.first ], map_index_to_node[ pair.second ]);
      edge_map[ edge ] = - matches.size();
    }

    // compute the MST of the graph
    std::vector<lemon::ListGraph::Edge> tree_edge_vec;
    lemon::kruskal(graph, edge_map, std::back_inserter(tree_edge_vec));

    Pair_Set mst;
    for (const lemon::ListGraph::Edge & edge : tree_edge_vec )
    {
      mst.insert(Pair(map_node_to_index[graph.u(edge)], map_node_to_index[graph.v(edge)]));
    }

    msts[ connected_component.first ] = mst;
  }

  return true;
}

} // namespace sfm
} // namespace openMVG
