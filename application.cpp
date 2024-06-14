/*
Author: Aaryan Sharma
CS 251 (Fall 2023) - Prof Drishika Dey
Project - 5 (Open Street Maps)
*/


// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
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

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;


const double INF = numeric_limits<double>::max();


/*
    Class Definitions:
*/
class VertexInfo {
public:
    long long vertexID;
    double distance;

    VertexInfo(long long v, double d) : vertexID(v), distance(d) {}

    bool operator>(const VertexInfo& other) const {
        return this->distance > other.distance;
    }
};


class EdgeInfo {
public:
    bool operator()(const VertexInfo& p1, const VertexInfo& p2) const {
        return p1.distance > p2.distance;
    }
};


/*
    Print Functions:
*/
void printInfo (BuildingInfo person1Building, BuildingInfo person2Building,
                BuildingInfo centerBuilding, Coordinates startNode1,
                Coordinates startNode2, Coordinates destinationNode) {

    cout << "Person 1's point:" << endl
         << " " << person1Building.Fullname << endl
         << " (" << person1Building.Coords.Lat << ", " << person1Building.Coords.Lon << ")\n";

    cout << "Person 2's point:" << endl
         << " " << person2Building.Fullname << endl
         << " (" << person2Building.Coords.Lat << ", " << person2Building.Coords.Lon << ")\n";

    cout << "Destination Building:" << endl
         << " " << centerBuilding.Fullname << endl
         << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")\n";

    cout << endl
         << "Nearest P1 node:" << endl
         << " " << startNode1.ID << endl
         << " (" << startNode1.Lat << ", " << startNode1.Lon << ")\n";

    cout << "Nearest P2 node:" << endl
         << " " << startNode2.ID << endl
         << " (" << startNode2.Lat << ", " << startNode2.Lon << ")\n";

    cout << "Nearest destination node:" << endl
         << " " << destinationNode.ID << endl
         << " (" << destinationNode.Lat << ", " << destinationNode.Lon << ")\n";
}



void printNew (BuildingInfo newBuildingCenter, Coordinates newdest){
    cout << "New Destination Building: \n";
    cout << " " << newBuildingCenter.Fullname << endl;
    cout << " (" << newBuildingCenter.Coords.Lat << ", " << newBuildingCenter.Coords.Lon << ")\n";
    cout << "Nearest Destination Node:\n" << " " << newdest.ID << endl << " (" << newdest.Lat << ", " << newdest.Lon << ")" << endl;
}


void printPath(vector<long long>& p) {

    cout << "Path: ";
    for (int i = 0; i < p.size() - 1; i++) {
        cout << p[i] << "->";
    }

    cout << p[p.size() - 1] << endl;
}


/*
    Helper Functions:
*/
bool searchHelp (const vector<BuildingInfo>& buildings,
                 const string& name, BuildingInfo& building, bool searchByAbbrev) {
    for (const auto& it : buildings) {
        if ((searchByAbbrev && it.Abbrev == name) || (!searchByAbbrev && it.Fullname.find(name) != string::npos)) {
            building = it;
            return true;
        }
    }
    return false;
}


BuildingInfo locatorHelp (const vector<BuildingInfo>& buildings,
                          double targetLat, double targetLong, const set<string>& unreachableBuildings) {
    double minDist = INF;
    int closestIndex = -1;

    for (size_t i = 0; i < buildings.size(); ++i) {
        if (unreachableBuildings.count(buildings[i].Fullname) > 0) {
            continue;
        }

        double buildLat = buildings[i].Coords.Lat;
        double buildLong = buildings[i].Coords.Lon;
        double distance = distBetween2Points(buildLat, buildLong, targetLat, targetLong);

        if (distance < minDist) {
            minDist = distance;
            closestIndex = i;
        }
    }

    return (closestIndex != -1) ? buildings[closestIndex] : BuildingInfo();
}


void reversePath (stack<long long>& pathStack, const map<long long, long long>& predecessors,
                  long long startVertex, long long currentVertex) {
    while (startVertex != currentVertex) {
        pathStack.push(currentVertex);
        currentVertex = predecessors.at(currentVertex);
    }
    pathStack.push(startVertex);
}


vector<long long> stackToPath (stack<long long>& pathStack) {
    vector<long long> path;
    while (!pathStack.empty()) {
        path.push_back(pathStack.top());
        pathStack.pop();
    }
    return path;
}


bool loadMapFile(XMLDocument& xmlDoc, const string& filename) {
    if (!LoadOpenStreetMap(filename, xmlDoc)) {
        cout << "**Error: unable to load open street map." << endl << endl;
        return false;
    }
    return true;
}


void printMapStats(const map<long long, Coordinates>& nodes,
                   const vector<FootwayInfo>& footways,
                   const vector<BuildingInfo>& buildings) {
    cout << "# of nodes: " << nodes.size() << endl;
    cout << "# of footways: " << footways.size() << endl;
    cout << "# of buildings: " << buildings.size() << endl;
    cout << endl;
}


void readMapData(XMLDocument& xmlDoc, map<long long, Coordinates>& nodes,
                 vector<FootwayInfo>& footways, vector<BuildingInfo>& buildings) {
    int nodeCount = ReadMapNodes(xmlDoc, nodes);
    int footwayCount = ReadFootways(xmlDoc, footways);
    int buildingCount = ReadUniversityBuildings(xmlDoc, nodes, buildings);

    assert(nodeCount == (int)nodes.size());
    assert(footwayCount == (int)footways.size());
    assert(buildingCount == (int)buildings.size());

    printMapStats(nodes, footways, buildings);
}



void buildGraph(graph<long long, double>& G, map<long long, Coordinates>& nodes,
                const vector<FootwayInfo>& footways) {
    for (const auto& node : nodes) {
        G.addVertex(node.first);
    }

    for (const auto& footway : footways) {
        for (size_t i = 0; i < footway.Nodes.size() - 1; i++) {
            double distance = distBetween2Points(nodes[footway.Nodes[i]].Lat, nodes[footway.Nodes[i]].Lon,
                                                 nodes[footway.Nodes[i + 1]].Lat, nodes[footway.Nodes[i + 1]].Lon);
            G.addEdge(footway.Nodes[i], footway.Nodes[i + 1], distance);
            G.addEdge(footway.Nodes[i + 1], footway.Nodes[i], distance);
        }
    }
}


void printStats(const graph<long long, double>& G) {
    cout << "# of vertices: " << G.NumVertices() << endl;
    cout << "# of edges: " << G.NumEdges() << endl;
    cout << endl;
}

/*
    Main driver functions:
*/

bool search (vector<BuildingInfo>& buildings, const string& name,
             BuildingInfo& building, const string& searchType = "Abbrev") {
    bool isAbbrevSearch = searchType == "Abbrev";

    if (searchHelp(buildings, name, building, isAbbrevSearch)) {
        return true;
    }

    if (isAbbrevSearch) {
        return searchHelp(buildings, name, building, false);
    }

    return false;
}

BuildingInfo locateCenter (vector<BuildingInfo>& buildings, double centerLat,
                           double centerLong, set<string> unreachableBuildings) {
    return locatorHelp(buildings, centerLat, centerLong, unreachableBuildings);
}


Coordinates locateNodes (map<long long, Coordinates>& Nodes, BuildingInfo Building1, vector<FootwayInfo> Footways){
    double minDist = INF;
    Coordinates ret;
    for(auto it: Footways){
        for(auto it2: it.Nodes){
            if(distBetween2Points(Building1.Coords.Lat, Building1.Coords.Lon, Nodes[it2].Lat, Nodes[it2].Lon) < minDist){
                minDist = distBetween2Points(Building1.Coords.Lat, Building1.Coords.Lon, Nodes[it2].Lat, Nodes[it2].Lon);
                ret = Nodes[it2];
            }
        }
    }
    return ret;
}


vector<long long> getPath(long long startVertex, const map<long long, long long>& predecessors, long long destination) {
    stack<long long> pathStack;
    reversePath(pathStack, predecessors, startVertex, destination);
    return stackToPath(pathStack);
}


// Dijkstra Algorithm Helpers:
void initDijk (graph<long long, double>& graph, map<long long, double>& distances,
               map<long long, long long>& predecessors,
               priority_queue<VertexInfo, vector<VertexInfo>, EdgeInfo>& unvisitedQueue, long long startId) {

    for (auto vertex : graph.getVertices()) {
        distances[vertex] = INF;
        predecessors[vertex] = 0;
        unvisitedQueue.push(VertexInfo(vertex, INF));
    }

    distances[startId] = 0;
    unvisitedQueue.push(VertexInfo(startId, 0));

}


void neighUpdate (graph<long long, double>& graph, long long currentVertex,
                  map<long long, double>& distances, map<long long, long long>& predecessors,
                  priority_queue<VertexInfo, vector<VertexInfo>, EdgeInfo>& unvisitedQueue) {

    for (auto neighbor : graph.neighbors(currentVertex)) {
        double weight;
        graph.getWeight(currentVertex, neighbor, weight);
        double newDistance = distances[currentVertex] + weight;

        if (newDistance < distances[neighbor]) {
            distances[neighbor] = newDistance;
            predecessors[neighbor] = currentVertex;
            unvisitedQueue.push(VertexInfo(neighbor, newDistance));
        }
    }
}


void mainDijk (graph<long long, double>& graph, map<long long, double>& distances,
               map<long long, long long>& predecessors,
               priority_queue<VertexInfo, vector<VertexInfo>, EdgeInfo>& unvisitedQueue,
               vector<long long>& visitedVertices) {

    while (!unvisitedQueue.empty()) {
        long long currentVertex = unvisitedQueue.top().vertexID;
        unvisitedQueue.pop();

        if (distances[currentVertex] == INF) {
            break;
        }
        if (find(visitedVertices.begin(), visitedVertices.end(), currentVertex) != visitedVertices.end()) {
            continue;
        }

        visitedVertices.push_back(currentVertex);
        neighUpdate(graph, currentVertex, distances, predecessors, unvisitedQueue);
    }
}


// Dijkstra's algorithm:
void Dijkstra(long long startID, graph<long long, double> G, map<long long, double>& distance,
              map<long long, long long>& predecessors) {

    vector<long long> visited;
    priority_queue<VertexInfo, vector<VertexInfo>, EdgeInfo> unvisitedQueue;

    initDijk(G, distance, predecessors, unvisitedQueue, startID);
    mainDijk(G, distance, predecessors, unvisitedQueue, visited);
}


void application(
        map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
        vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
    string person1Building, person2Building;

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);
        BuildingInfo Building1, Building2;
        if(!search(Buildings, person1Building, Building1)){
            cout << "Person 1's building not found" << endl;
        }
        else{
            if(!search(Buildings, person2Building, Building2)){
                cout << "Person 2's building not found" << endl;
            }
            else{
                Coordinates center = centerBetween2Points(Building1.Coords.Lat, Building1.Coords.Lon, Building2.Coords.Lat, Building2.Coords.Lon);
                double centerLat =center.Lat;
                double centerLong = center.Lon;
                set<string> unreachableBuildings;
                bool path = false;

                while(!path){
                    BuildingInfo BuildingCenter = locateCenter(Buildings, centerLat, centerLong, unreachableBuildings);
                    Coordinates start1 = locateNodes(Nodes, Building1, Footways);
                    Coordinates start2 = locateNodes(Nodes, Building2, Footways);
                    Coordinates dest = locateNodes(Nodes, BuildingCenter, Footways);
                    int unreachable = unreachableBuildings.size();
                    if(unreachable == 0){
                        cout << "\n";
                        printInfo(Building1,
                                  Building2, BuildingCenter,
                                  start1, start2, dest);

                    }
                    else{
                        printNew(BuildingCenter, dest);
                    }
                    map<long long, double> distancep1, distancep2;
                    map<long long, long long> predecessorsp1, predecessorsp2;
                    Dijkstra(start1.ID, G, distancep1, predecessorsp1);
                    if(distancep1[start2.ID] == INF){
                        cout << "\nSorry destination unreachable.\n";
                        break;
                    }
                    Dijkstra(start2.ID, G, distancep2, predecessorsp2);
                    if(distancep1[dest.ID] >= INF || distancep2[dest.ID] >= INF){
                        cout << "At least one person was unable to reach the destination building.";
                        cout << " Finding next closest building..." << endl << endl;
                        unreachableBuildings.insert(BuildingCenter.Fullname);
                        continue;
                    }

                    path = true;

                    vector<long long> path1 = getPath(start1.ID, predecessorsp1, dest.ID);
                    vector<long long> path2 = getPath(start2.ID, predecessorsp2, dest.ID);

                    cout << "\nPerson 1's distance to dest: " << distancep1[dest.ID] << " miles\n";
                    printPath(path1);
                    cout << "\nPerson 2's distance to dest: " << distancep2[dest.ID] << " miles\n";
                    printPath(path2);
                }

            }
        }
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
    }
    cout << endl;
}

int main() {
    graph<long long, double> graph;
    map<long long, Coordinates> nodes;
    vector<FootwayInfo> footways;
    vector<BuildingInfo> buildings;
    XMLDocument xmlDoc;

    cout << "** Navigating UIC open street map **" << endl << endl;
    cout << std::setprecision(8);

    string defaultFilename = "map.osm";
    string filename;

    cout << "Enter map filename> " << endl;
    getline(cin, filename);

    if (filename.empty()) {
        filename = defaultFilename;
    }

    if (!loadMapFile(xmlDoc, filename)) {
        return 0;
    }

    readMapData(xmlDoc, nodes, footways, buildings);
    buildGraph(graph, nodes, footways);
    printStats(graph);
    application(nodes, footways, buildings, graph);

    cout << endl << "** Done **" << endl;
    return 0;
}