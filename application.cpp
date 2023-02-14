// application.cpp <Starter Code>
// <Jack Li>
//
// University of Illinois at Chicago
// CS 251: Fall 2021
// Project #7 - Openstreet Maps
//
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <stack>
#include <string>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

const double INF = numeric_limits<double>::max();

//
// Implement your creative component application here
// TO DO: add arguments
//
void creative(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
              vector<BuildingInfo>& Buildings, graph<long long, double> G) {}

// _checkBuilding
// helper function that return true if the building that is being check is
// inside of the unreachable buildings
bool _checkBuilding(set<string> failedBuildings, string building) {
  for (const auto& failedb : failedBuildings) {
    if (building == failedb) {
      return false;
    }
  }
  return true;
}

// searchBuilding
// This function seach the Buildings vector if any of the buildings matches the
// inputed string
BuildingInfo searchBuilding(string query, vector<BuildingInfo>& Buildings) {
  for (const auto& val : Buildings) {
    if ((val.Abbrev.find(query) != string::npos ||
         val.Fullname.find(query) != string::npos)) {
      return val;
    }
  }
  BuildingInfo none;
  none.Fullname = "";
  return none;
}

// findCenter
// This function calculates the center between two buildings, then look for the
// closest building to the center point
BuildingInfo findCenter(BuildingInfo building1, BuildingInfo building2,
                        vector<BuildingInfo>& Buildings,
                        set<string>& failedBuildings) {
  double min = INF;
  BuildingInfo minDistBuilding;
  Coordinates midpoint =
      centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon,
                           building2.Coords.Lat, building2.Coords.Lon);
  for (const auto& val : Buildings) {
    double dist = distBetween2Points(midpoint.Lat, midpoint.Lon, val.Coords.Lat,
                                     val.Coords.Lon);
    if (dist < min && _checkBuilding(failedBuildings, val.Fullname)) {
      min = dist;
      minDistBuilding = val;
    }
  }

  return minDistBuilding;
}

// nearestNode
// This function returns the closest point on the footway to the given building
Coordinates nearestNode(BuildingInfo b, map<long long, Coordinates>& nodes,
                        vector<FootwayInfo>& FootWays) {
  double min = INF;
  double dist = 0;
  Coordinates minNode;
  for (const auto& val : FootWays) {
    for (const auto& node : val.Nodes) {
      dist = distBetween2Points(b.Coords.Lat, b.Coords.Lon, nodes[node].Lat,
                                nodes[node].Lon);
      if (dist < min) {
        min = dist;
        minNode = nodes[node];
      }
    }
  }

  return minNode;
}

// prioritize class
// compare function for prioritized queue
class prioritize {
 public:
  bool operator()(const pair<long long, double>& p1,
                  const pair<long long, double>& p2) const {
    return p1.second > p2.second;
  }
};

// dijkstra
// This function returns a map of closest distance to different vertices from
// the given starting point and return a map of predesscor by reference by using
// dijkstra's method
map<long long, double> dijkstra(Coordinates startV,
                                map<long long, Coordinates>& Nodes,
                                graph<long long, double>& G,
                                map<long long, long long>& pred) {
  map<long long, double> dist;
  set<long long> visited;
  priority_queue<pair<long long, double>, vector<pair<long long, double>>,
                 prioritize>
      unvisited;
  for (const auto& pair : Nodes) {
    dist[pair.first] = INF;
    pred[pair.first] = -1;
    unvisited.push(make_pair(pair.first, INF));
  }

  dist[startV.ID] = 0;
  pred[startV.ID] = 0;
  unvisited.push(make_pair(startV.ID, 0));

  while (!unvisited.empty()) {
    long long cur = unvisited.top().first;
    double curW = unvisited.top().second;
    unvisited.pop();

    if (curW == INF) {
      break;
    } else if (visited.count(cur)) {
      continue;
    } else {
      visited.insert(cur);
    }
    set<long long> adjV = G.neighbors(cur);

    for (const auto& val : adjV) {
      double edgeWeight = 0;
      if (!G.getWeight(cur, val, edgeWeight)) {
        continue;
      }

      double altPathDist = dist[cur] + edgeWeight;

      if (altPathDist < dist[val]) {
        dist[val] = altPathDist;
        pred[val] = cur;
        unvisited.push(make_pair(val, altPathDist));
      }
    }
  }

  return dist;
}

// getPath
// This function returns a vector of IDs that is the shortest path to the given
// ending vertice from a specific distance map
vector<long long> getPath(map<long long, long long>& pred, long long endV) {
  vector<long long> path;
  stack<long long> s;
  long long cur = endV;
  while (cur != 0) {
    s.push(cur);
    cur = pred[cur];
  }

  while (!s.empty()) {
    cur = s.top();
    s.pop();
    path.push_back(cur);
  }

  return path;
}

// OutputInfo
// This function outputs the Building's information
void OutputInfo(BuildingInfo Building) {
  cout << " " << Building.Fullname << endl;
  cout << " (" << Building.Coords.Lat << ", " << Building.Coords.Lon << ")\n";
}

// OutputInfo
// Overloaded function for OutputInfo that outputs the coordinates's information
void OutputInfo(Coordinates Cor) {
  cout << " " << Cor.ID << endl;
  cout << " (" << Cor.Lat << ", " << Cor.Lon << ")\n";
}

// printPath
// This function Outputs the path by using a given ID vector
void printPath(vector<long long> path) {
  cout << "Path: ";
  cout << path[0];
  for (size_t i = 1; i < path.size(); i++) {
    cout << "->" << path[i];
  }
  cout << endl << endl;
}

void application(map<long long, Coordinates>& Nodes,
                 vector<FootwayInfo>& Footways, vector<BuildingInfo>& Buildings,
                 graph<long long, double>& G) {
  string person1Building, person2Building;
  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    set<string> failedBuildings;
    bool findSecondCenter = false;
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Search Buildings 1 and 2
    BuildingInfo Building1 = searchBuilding(person1Building, Buildings);
    BuildingInfo Building2 = searchBuilding(person2Building, Buildings);
    // if any of the buildings are not found, enter again
    if (Building1.Fullname == "") {
      cout << "Person 1's building not found" << endl;
      cout << endl;
      cout
          << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    }
    if (Building2.Fullname == "") {
      cout << "Person 2's building not found" << endl;
      cout
          << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      continue;
    }
    cout << endl << "Person 1's point:" << endl;
    OutputInfo(Building1);
    cout << "Person 2's point:" << endl;
    OutputInfo(Building2);

    // first run the steps, then repeat until the center building is valid
    do {
      // Locate Center Building
      BuildingInfo centerBuilding =
          findCenter(Building1, Building2, Buildings, failedBuildings);
      if (!findSecondCenter) {
        cout << "Destination Building:" << endl;
        OutputInfo(centerBuilding);
        cout << endl;
      } else {
        cout << "New destination building:" << endl;
        OutputInfo(centerBuilding);
      }

      // Find Nearest Nodes from buildings 1, 2 & Center
      Coordinates b1Node = nearestNode(Building1, Nodes, Footways);
      Coordinates b2Node = nearestNode(Building2, Nodes, Footways);
      Coordinates bcNode = nearestNode(centerBuilding, Nodes, Footways);
      if (!findSecondCenter) {
        cout << "Nearest P1 node:\n";
        OutputInfo(b1Node);
        cout << "Nearest P2 node:\n";
        OutputInfo(b2Node);
        cout << "Nearest destination node:\n";
        OutputInfo(bcNode);
        cout << endl;
      } else {
        cout << "Nearest destination node:\n";
        OutputInfo(bcNode);
        cout << endl;
      }

      // Run Dijkstra’s Algorithm
      map<long long, long long> predB1;
      map<long long, long long> predB2;
      map<long long, double> distB1 = dijkstra(b1Node, Nodes, G, predB1);
      map<long long, double> distB2 = dijkstra(b2Node, Nodes, G, predB2);
      // If Buildings1 cannot reach Building2, enter the buildings again
      // Else if anyone cannot reach the center building, find the next closest
      // building
      if (distB1[b2Node.ID] >= INF) {
        cout << "Sorry, destination unreachable." << endl;
        break;
      } else if (distB1[bcNode.ID] >= INF || distB2[bcNode.ID] >= INF) {
        cout << "At least one person was unable to reach the destination "
                "building. Finding next closest building..."
             << endl
             << endl;
        failedBuildings.insert(centerBuilding.Fullname);
        findSecondCenter = true;
      } else {
        // Print path
        vector<long long> path1 = getPath(predB1, bcNode.ID);
        vector<long long> path2 = getPath(predB2, bcNode.ID);
        cout << "Person 1's distance to dest: " << distB1[bcNode.ID]
             << " miles\n";
        printPath(path1);
        cout << "Person 2's distance to dest: " << distB2[bcNode.ID]
             << " miles\n";
        printPath(path2);
        findSecondCenter = false;
      }
    } while (findSecondCenter);

    // another navigation?
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
    findSecondCenter = false;
  }
}

int main() {
  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates> Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo> Footways;
  // info about each building, in no particular order
  vector<BuildingInfo> Buildings;
  XMLDocument xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  graph<long long, double> G;

  // add vertices
  for (const auto& pair : Nodes) {
    G.addVertex(pair.second.ID);
  }
  // add edges
  for (const auto& val : Footways) {
    for (size_t i = 0; i < val.Nodes.size() - 1; i++) {
      double dist = distBetween2Points(
          Nodes[val.Nodes[i]].Lat, Nodes[val.Nodes[i]].Lon,
          Nodes[val.Nodes[i + 1]].Lat, Nodes[val.Nodes[i + 1]].Lon);
      G.addEdge(val.Nodes[i], val.Nodes[i + 1], dist);
      G.addEdge(val.Nodes[i + 1], val.Nodes[i], dist);
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  //
  // Menu
  //
  string userInput;
  cout << "Enter \"a\" for the standard application or "
       << "\"c\" for the creative component application> ";
  getline(cin, userInput);
  if (userInput == "a") {
    application(Nodes, Footways, Buildings, G);
  } else if (userInput == "c") {
    // TO DO: add arguments
    creative(Nodes, Footways, Buildings, G);
  }
  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
