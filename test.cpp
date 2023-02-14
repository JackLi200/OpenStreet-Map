#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <string>
#include <fstream>

#include "graph.h"

using namespace std;

int main() {
  graph<string,int> g;
  g.addVertex("A");
  g.addVertex("B");
  g.addVertex("C");
  g.addEdge("A", "B", 6);
  g.addEdge("A", "B", 7);
  g.addEdge("B", "A", 7);
  g.addEdge("A", "C", 6);
  g.addEdge("A", "D", 6);
  g.addEdge("D", "A", 6);
  g.addEdge("B", "A", 11);
  g.dump(cout);
  graph<string,int> g1(g);
  graph<string,int> g2 = g;

  g1.dump(cout);
  g2.dump(cout);
}