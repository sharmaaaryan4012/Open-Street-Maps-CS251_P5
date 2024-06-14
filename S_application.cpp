/*
 * Project 6
 * CS251 - Spring 2023
 * Author: Aarav Surkatha
 * Starter Code By: Prof Adam T Koehler
 * Project Original Variartion By:
 * Joe Hummel, PhD
 * University of Illinois at Chicago
 */
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <stack>
#include <set>
#include <limits>
#include <cassert>
#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "algorithm"
#include "cmath"
using namespace std;
using namespace tinyxml2;
typedef graph<long long, double> Graph;
typedef map<long long, Coordinates> NodeMap;
const double INF = numeric_limits<double>::max();
class prioritize{
public:
    bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const {

        return p1.second > p2.second;
    }
};
typedef priority_queue <pair<long long, double>, vector<pair<long long, double>>,prioritize> PriorityQueue;
typedef vector<long long> Path;
// GetNearestNode loops through the footway vector and compares the distance between the building
// and the nodes in the footway vector. It returns the nearest node to the building.
long long GetNearestNode(BuildingInfo building, vector<FootwayInfo>& footway, map<long long, Coordinates>& coordinate)
{
    //keeping track of minimum distance , intialized to infinity
    double MinimumDistance = INF;
    long long NearestPoint;
    for (auto& idx : footway)
    {
        for (long unsigned int i = 0; i < idx.Nodes.size(); i++) {

            double distance = distBetween2Points(building.Coords.Lat, building.Coords.Lon,coordinate[idx.Nodes[i]].Lat, coordinate[idx.Nodes[i]].Lon);
            if (distance < MinimumDistance)
            {
                MinimumDistance = distance;
                NearestPoint = idx.Nodes[i];
            }
        }
    }
    return NearestPoint;
}
//FindBuilding loops through the building vector and compares the building name with the building name entered by the user.
//If the building name is found, it returns true and the building info is stored in the buildingInfo variable.
bool FindBuilding(string buildingName, vector<BuildingInfo>& buildingList, BuildingInfo& buildingInfo)
{
    for (long unsigned int i = 0; i < buildingList.size(); i++)
    {
        if (buildingList[i].Fullname.find(buildingName) != string::npos)
        {
            buildingInfo = buildingList[i];
            return true;
        }
    }
    return false;
}
//FindNearestBuilding loops through the building vector and compares the distance between the midpoint and the building.
//It returns the nearest building to the midpoint.
BuildingInfo FindNearestBuilding(Coordinates midpoint, vector<BuildingInfo>& Buildings, set<string>& unreachable)
{
    //keeping track of minimum distance , intialized to infinity
    double MinumumDistance = INF;
    BuildingInfo NearestBuilding;

    for (auto& b : Buildings) {

        if (unreachable.count(b.Fullname) != 0) {
            continue;
        }
        double distance = distBetween2Points(midpoint.Lat, midpoint.Lon, b.Coords.Lat, b.Coords.Lon);

        if (distance < MinumumDistance) {

            MinumumDistance = distance;
            NearestBuilding = b;
        }
    }
    return NearestBuilding;
}
//GetPath loops through the predecessor map and stores the path in a vector.
Path GetPath(long long startVertex, map<long long, long long>& predStorage, long long destination) {

    Path VectorOfPath;
    stack<long long> StackOfPath;

    StackOfPath.push(destination);
    long long curr = destination;

    while (startVertex != curr) {

        curr = predStorage[curr];
        StackOfPath.push(curr);
    }

    while (!StackOfPath.empty()) {

        curr = StackOfPath.top();
        VectorOfPath.push_back(curr);
        StackOfPath.pop();
    }

    return VectorOfPath;
}
Path SolveByDijkstra(Graph G, long long startV, map<long long, double>& distance, map<long long, long long>& predecessors)
{
    Path visited;
    PriorityQueue unvisited;

    for (auto a : G.getVertices())
    {
        distance[a] = INF;
        predecessors[a] = 0;
        unvisited.push(make_pair(a, INF));
    }

    distance[startV] = 0;
    unvisited.push(make_pair(startV, 0));

    while (!unvisited.empty())
    {
        long long topV = unvisited.top().first;
        unvisited.pop();

        if (distance[topV] == INF)
        {
            break;
        }
        else if (count(visited.begin(), visited.end(), topV) > 0)
        {
            continue;
        }
        else
        {
            visited.push_back(topV);
        }

        for (auto neighbor : G.neighbors(topV))
        {
            double edgeWeight;
            G.getWeight(topV, neighbor, edgeWeight);
            double alternative = distance[topV] + edgeWeight;
            if (alternative < distance[neighbor])
            {
                distance[neighbor] = alternative;
                unvisited.push(make_pair(neighbor, alternative));
                predecessors[neighbor] = topV;
            }
        }
    }
    return visited;
}
void application(NodeMap& Nodes, vector<FootwayInfo>& Footways, vector<BuildingInfo>& Buildings, Graph& G)
{
    BuildingInfo Point1, Point2;
    string firstBuidling, secondBuilding;
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, firstBuidling);
    while (firstBuidling != "#")
    {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, secondBuilding);
        if (!FindBuilding(firstBuidling, Buildings, Point1)) {
            cout << "Person 1's building not found" << endl;
            cout << "Enter person 1's building (partial name or abbreviation), or #> ";
            getline(cin, firstBuidling);
            continue;
        }
        if (!FindBuilding(secondBuilding, Buildings, Point2)) {
            cout << "Person 2's building not found" << endl;
            cout << "Enter person 1's building (partial name or abbreviation), or #> ";
            getline(cin, firstBuidling);
            continue;
        }
        Coordinates Midpoint = centerBetween2Points(Point1.Coords.Lat,Point1.Coords.Lon, Point2.Coords.Lat, Point2.Coords.Lon);
        set <string> unreachable;
        bool PathValid = true;
        while (PathValid)
        {
            BuildingInfo midBuilding = FindNearestBuilding(Midpoint, Buildings, unreachable);
            long long NearNodeID1 = GetNearestNode(Point1, Footways, Nodes);
            long long NearNodeID2 = GetNearestNode(Point2, Footways, Nodes);
            long long NearNodeIDCen = GetNearestNode(midBuilding, Footways, Nodes);
            if (unreachable.size() == 0)
            {
                cout << endl;
                //Building Print
                cout << "Person 1's point:" << endl;
                cout << " " << Point1.Fullname << endl;
                cout << " (" << Point1.Coords.Lat << ", " << Point1.Coords.Lon << ")" << endl;
                cout << "Person 2's point:" << endl;
                cout << " " << Point2.Fullname << endl;
                cout << " (" << Point2.Coords.Lat << ", " << Point2.Coords.Lon << ")" << endl;
                cout << "Destination Building:" << endl;
                cout << " " << midBuilding.Fullname << endl;
                cout << " (" << midBuilding.Coords.Lat << ", " << midBuilding.Coords.Lon << ")" << endl;
                cout << endl;
                //Nearest Nodes Print
                cout << "Nearest P1 node:" << endl;
                cout << " " << NearNodeID1 << endl;
                cout << " (" << Nodes[NearNodeID1].Lat << ", " << Nodes[NearNodeID1].Lon << ")" << endl;
                cout << "Nearest P2 node:" << endl;
                cout << " " << NearNodeID2 << endl;
                cout << " (" << Nodes[NearNodeID2].Lat << ", " << Nodes[NearNodeID2].Lon << ")" << endl;
                cout << "Nearest destination node:" << endl;
                cout << " " << NearNodeIDCen << endl;
                cout << " (" << Nodes[NearNodeIDCen].Lat << ", " << Nodes[NearNodeIDCen].Lon << ")" << endl << endl;
            }
            else
            {
                //Destination Building Print
                cout << "New destination building:" << endl;
                cout << " " << midBuilding.Fullname << endl;
                cout << " (" << midBuilding.Coords.Lat << ", " << midBuilding.Coords.Lon << ")" << endl;
                cout << "Nearest destination node:" << endl;
                cout << " " << NearNodeIDCen << endl;
                cout << " (" << Nodes[NearNodeIDCen].Lat << ", " << Nodes[NearNodeIDCen].Lon << ")" << endl << endl;
            }
            map<long long, double> distances1, distances2;
            map<long long, long long> PredMapA, PredMapB;
            Path p1 = SolveByDijkstra(G, NearNodeID1, distances1, PredMapA);
            if (distances1[NearNodeID2] >= INF)
            {
                cout << "Sorry, destination unreachable." << endl;
                break;
            }
            Path p2 = SolveByDijkstra(G, NearNodeID2, distances2, PredMapB);
            if (distances1[NearNodeIDCen] >= INF || distances2[NearNodeIDCen] >= INF)
            {
                cout << "At least one person was unable to reach the destination building.";
                cout << " Finding next closest building..." << endl << endl;
                unreachable.insert(midBuilding.Fullname);
            }
            else
            {
                PathValid = false;
                Path path1 = GetPath(NearNodeID1, PredMapA, NearNodeIDCen);
                Path path2 = GetPath(NearNodeID2, PredMapB, NearNodeIDCen);
                cout << "Person 1's distance to dest: " << distances1[NearNodeIDCen];
                cout << " miles" << endl;
                cout << "Path: ";
                for (long unsigned int i = 0; i < path1.size() - 1; i++)
                {
                    cout << path1[i] << "->";
                }
                cout << path1[path1.size() - 1] << endl;
                cout << endl;
                cout << "Person 2's distance to dest: " << distances2[NearNodeIDCen];
                cout << " miles" << endl;
                cout << "Path: ";
                for (long unsigned int i = 0; i < path2.size() - 1; i++)
                {
                    cout << path2[i] << "->";
                }
                cout << path2[path2.size() - 1] << endl;
            }
        }
        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, firstBuidling);
    }
}
int main()
{
    Graph G;
    NodeMap Nodes;
    vector<FootwayInfo> Footways;
    vector<BuildingInfo> Buildings;
    XMLDocument xmldoc;
    cout << "** Navigating UIC open street map **" << endl;
    cout << endl;
    cout << std::setprecision(8);
    string DefaultFilename = "map.osm";
    string filename;
    cout << "Enter map filename> ";
    getline(cin, filename);
    if (filename == "")
    {
        filename = DefaultFilename;
    }
    if (!LoadOpenStreetMap(filename, xmldoc))
    {
        cout << "**Error: unable to load open street map." << endl;
        cout << endl;
        return 0;
    }
    int nodeCount = ReadMapNodes(xmldoc, Nodes);
    int footwayCount = ReadFootways(xmldoc, Footways);
    int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);
    assert(nodeCount == (int)Nodes.size());
    assert(footwayCount == (int)Footways.size());
    assert(buildingCount == (int)Buildings.size());
    cout << endl;
    cout << "# of nodes: " << Nodes.size() << endl;
    cout << "# of footways: " << Footways.size() << endl;
    cout << "# of buildings: " << Buildings.size() << endl;
    for (auto& a : Nodes)
    {
        G.addVertex(a.first);
    }
    for (auto& b: Footways)
    {
        for (int i = 0; i < ((int)b.Nodes.size())-1; i++)
        {
            double weight = distBetween2Points(Nodes[b.Nodes[i]].Lat, Nodes[b.Nodes[i]].Lon, Nodes[b.Nodes[i+1]].Lat, Nodes[b.Nodes[i+1]].Lon);
            G.addEdge(b.Nodes[i], b.Nodes[i+1], weight);
            G.addEdge(b.Nodes[i + 1], b.Nodes[i], weight);
        }
    }
    cout << "# of vertices: " << G.NumVertices() << endl;
    cout << "# of edges: " << G.NumEdges() << endl;
    cout << endl;
    application(Nodes, Footways, Buildings, G);
    cout << "** Done **" << endl;
    return 0;
}