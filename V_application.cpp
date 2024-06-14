// application.cpp <Starter Code>
// <Your name>
//
//
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

bool searchBuilding(vector<BuildingInfo>& Buildings, string name, BuildingInfo& building, string x = "Abbrev"){
	if(x == "Abbrev"){
		for(auto it: Buildings){
			if(it.Abbrev == name){
				building = it;
				return true;
			}
		}
		return searchBuilding(Buildings, name, building, "Name");
	}
	for(auto it: Buildings){
		if(it.Fullname.find(name) != string::npos){
			building = it;
			return true;
		}
	}
	return false;
}

BuildingInfo findCenterBuilding(vector<BuildingInfo>& Buildings, double centerLat, double centerLong, set<string> unreachableBuildings){
	double minDist = INF;
	int dest = 0;
	for(size_t it = 0; it < Buildings.size(); it++){
		if(unreachableBuildings.count(Buildings[it].Fullname) > 0){
			continue;
		}
		double buildLat = Buildings[it].Coords.Lat;
		double buildLong = Buildings[it].Coords.Lon;
		if(distBetween2Points(buildLat, buildLong, centerLat, centerLong) < minDist){
			minDist = distBetween2Points(buildLat, buildLong, centerLat, centerLong);
			dest = it;
		}
	}
	return Buildings[dest];
}

Coordinates findNodes(map<long long, Coordinates>& Nodes, BuildingInfo Building1, vector<FootwayInfo> Footways){
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

void printBuilding(BuildingInfo Building1, BuildingInfo Building2, BuildingInfo BuildingCenter){
	cout << "Person 1's point:" << endl << " " << Building1.Fullname << endl << 
		" ("<< Building1.Coords.Lat << ", " << Building1.Coords.Lon << ")\n";
		cout << "Person 2's point:" << endl << " " << Building2.Fullname << endl << 
		" ("<< Building2.Coords.Lat << ", " << Building2.Coords.Lon << ")\n";
		cout << "Destination Building:" << endl << " " << BuildingCenter.Fullname << endl << 
		" ("<< BuildingCenter.Coords.Lat << ", " << BuildingCenter.Coords.Lon << ")\n";
}

void printNodes(Coordinates start1, Coordinates start2, Coordinates dest){
	cout << endl << "Nearest P1 node:" << endl << " " << start1.ID << endl << " (" << start1.Lat << ", " << start1.Lon << ")\n";
		cout << "Nearest P2 node:" << endl << " " << start2.ID << endl << " (" << start2.Lat << ", " << start2.Lon << ")\n";
		cout << "Nearest destination node:" << endl << " " << dest.ID << endl << " (" << dest.Lat << ", " << dest.Lon << ")\n";
}

void printNew(BuildingInfo newBuildingCenter, Coordinates newdest){
	cout << "New Destination Building: \n";
	cout << " " << newBuildingCenter.Fullname << endl;
	cout << " (" << newBuildingCenter.Coords.Lat << ", " << newBuildingCenter.Coords.Lon << ")\n";
	cout << "Nearest Destination Node:\n" << " " << newdest.ID << endl << " (" << newdest.Lat << ", " << newdest.Lon << ")" << endl;
}

struct func{
	typedef pair<long long, double> Edge;
	bool operator()(const Edge& p1, const Edge& p2) const{
		return p1.second > p2.second;
	}
};

void Dijkstra(long long startID, graph<long long, double> G, map<long long, double>& distance, map<long long, long long>& predecessors){
	vector<long long> visited;
	typedef pair<long long, double> Edge;
	priority_queue<Edge, vector<Edge>, func> unvisitedQueue;
	for(auto vert: G.getVertices()){
		distance[vert] = INF;
		predecessors[vert] = 0;
		pair temp = {vert, INF};
		unvisitedQueue.push(temp);
	}
	distance[startID] = 0;
	pair temp = {startID, 0};
	unvisitedQueue.push(temp);
	while(unvisitedQueue.size()!=0){
		long long currVertex = unvisitedQueue.top().first;
		unvisitedQueue.pop();
		if(distance[currVertex] == INF){
			break;
		}
		else if(count(visited.begin(), visited.end(), currVertex) > 0){
			continue;
		}
		visited.push_back(currVertex);
		for(auto it: G.neighbors(currVertex)){
			double weight;
			G.getWeight(currVertex, it, weight);
			double newDistance = distance[currVertex] + weight;
			if(newDistance < distance[it]){
				distance[it] = newDistance;
				predecessors[it] = currVertex;
				Edge edge = {it, newDistance};
				unvisitedQueue.push(edge);
			}
		}
	}

}

void getPath(
	vector<long long>& path1, vector<long long>& path2, long long start1ID, long long start2ID,
 	map<long long, long long> p1, map<long long, long long> p2, long long destID) {
	stack<long long> revP1;
	stack<long long> revP2;	
	revP1.push(destID);
	revP2.push(destID);
	long long top1 = destID, top2 = destID;
	while(top1 != start1ID){
		top1 = p1[top1];
		revP1.push(top1);
	}
	while(top2 != start2ID){
		top2 = p2[top2];
		revP2.push(top2);
	}
	while(!revP1.empty() ){
		top1 = revP1.top();
		path1.push_back(top1);
		revP1.pop();
	}
	while(!revP2.empty()){
		top2 = revP2.top();
		path2.push_back(top2);		
		revP2.pop();
	}
}

void printPath(vector<long long> path){
	cout << "Path: ";
	for(int node = 0; node < path.size() - 1; node++){
		cout << path[node] << "->";
	}
	cout << path[path.size() - 1]  << "\n";
}
//
// Implement your standard application here
//
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
		if(!searchBuilding(Buildings, person1Building, Building1)){
			cout << "Person 1's building not found" << endl;	 	
		}
		else{
			if(!searchBuilding(Buildings, person2Building, Building2)){
				cout << "Person 2's building not found" << endl;
			}
			else{
				Coordinates center = centerBetween2Points(Building1.Coords.Lat, Building1.Coords.Lon, Building2.Coords.Lat, Building2.Coords.Lon);
				double centerLat =center.Lat;
				double centerLong = center.Lon;
				set<string> unreachableBuildings;
				bool pathExists = false;
				while(!pathExists){
					BuildingInfo BuildingCenter = findCenterBuilding(Buildings, centerLat, centerLong, unreachableBuildings);
					Coordinates start1 = findNodes(Nodes, Building1, Footways);
					Coordinates start2 = findNodes(Nodes, Building2, Footways);
					Coordinates dest = findNodes(Nodes, BuildingCenter, Footways);
					int unreachable = unreachableBuildings.size();
					if(unreachable == 0){
						cout << "\n";
						printBuilding(Building1, Building2, BuildingCenter);
						printNodes(start1, start2, dest);
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
					pathExists = true;
					vector<long long> path1, path2;
					getPath(path1, path2, start1.ID, start2.ID, predecessorsp1, predecessorsp2, dest.ID);
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
	graph<long long, double> G;

	// maps a Node ID to it's coordinates (lat, lon)
	map<long long, Coordinates>  Nodes;
	// info about each footway, in no particular order
	vector<FootwayInfo>          Footways;
	// info about each building, in no particular order
	vector<BuildingInfo>         Buildings;
	XMLDocument                  xmldoc;

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

	for(auto it: Nodes){
		G.addVertex(it.first);
	}
	for(auto it: Footways){
		for(size_t i = 0; i < it.Nodes.size()-1; i++){
			G.addEdge(it.Nodes[i], it.Nodes[i+1], distBetween2Points(Nodes[it.Nodes[i]].Lat, Nodes[it.Nodes[i]].Lon, Nodes[it.Nodes[i+1]].Lat, Nodes[it.Nodes[i+1]].Lon));
			G.addEdge(it.Nodes[i+1], it.Nodes[i], distBetween2Points(Nodes[it.Nodes[i]].Lat, Nodes[it.Nodes[i]].Lon, Nodes[it.Nodes[i+1]].Lat, Nodes[it.Nodes[i+1]].Lon));
		}
	}

  //
  // TO DO: build the graph, output stats:
  //
	cout << "# of vertices: " << G.NumVertices() << endl;
	cout << "# of edges: " << G.NumEdges() << endl;
	cout << endl;

	// Execute Application
	application(Nodes, Footways, Buildings, G);

	//
	// done:
	//
	cout << endl << "** Done **" << endl;
	return 0;
}
