// graph.h <Starter Code>
// < Jack Li>
//
// Basic graph class using adjacency list representation with no limit.
//
// University of Illinois at Chicago
// CS 251: Fall 2021
// Project #7 - Openstreet Maps
//

#pragma once

#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

using namespace std;

template <typename VertexT, typename WeightT>
class graph {
 private:
  map<VertexT, map<VertexT, WeightT>> gMap;

 public:
  // constructor
  graph() {}

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const { return gMap.size(); }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;
    for (const auto &pair : gMap) {
      count += pair.second.size();
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph, if v already in the graph return false else
  // return true
  //
  bool addVertex(VertexT v) {
    if (gMap.count(v)) {
      return false;
    } else {
      map<VertexT, WeightT> edgeMap;
      gMap.emplace(v, edgeMap);
      return true;
    }
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    if (gMap.count(from)) {
      if (!gMap.count(to)) {
        return false;
      } else {
        gMap.at(from)[to] = weight;
        return true;
      }
    } else {
      return false;
    }
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT &weight) const {
    if (gMap.count(from)) {
      if (gMap.at(from).count(to)) {
        weight = gMap.at(from).at(to);
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;

    if (gMap.count(v)) {
      for (const auto &pair : gMap.at(v)) {
        S.insert(pair.first);
      }
    } else {
      return S;
    }

    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> v;
    for (const auto &pair : gMap) {
      v.push_back(pair.first);
    }
    return v;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  void dump(ostream &output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    for (const auto &pair : gMap) {
      output << pair.first << ": ";
      for (const auto &pair : pair.second) {
        output << "(" << pair.first << "," << pair.second << ") ";
      }
      output << endl;
    }
    output << "**************************************************" << endl;
  }

  graph(const graph<VertexT, WeightT>& other) {
    gMap = other.gMap;
  }

  graph& operator=(const graph<VertexT, WeightT>& other) {
    gMap = other.gMap;
  }
};
