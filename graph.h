/*
Author: Aaryan Sharma
CS 251 (Fall 2023) - Prof Drishika Dey
Project - 5 (Open Street Maps)
*/


// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
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

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include "map"
using namespace std;

template<typename VertexT, typename WeightT>
class graph {
private:
    int numEdges = 0;

    struct EdgeData{
        bool EdgeExists;
        VertexT  Vertex;
        WeightT  Weight;

        EdgeData() {
            EdgeExists = false;  // initially no edge, and no weight
        }
    };


    map<VertexT,vector<EdgeData>> AdjMatrix;
    vector<VertexT>  Vertices;


    int _LookupVertex(VertexT v) const {
        for (int i = 0; i < this->NumVertices(); ++i) {
            if (this->Vertices[i] == v)  // already in the graph:
            return i;
        }

        return -1;
    }


    bool isPresentVertex (VertexT v) const {                                                                            // This is a helper function, which verifies the existence of vertex "v" in our graph.
        if(AdjMatrix.count(v) == 0) {
            return false;
        }
        return true;
    }


    bool isPresent (VertexT from, VertexT to) const {                                                                   // This is a helper function, which verifies whether both vertices exist in our graph.
        if(AdjMatrix.count(from) == 0 || AdjMatrix.count(to) == 0) {
            return false;
        }
        return true;
    }

public:
    graph() {}                                                                                                          // Default constructor.


    int NumVertices() const {                                                                                           // This function returns the number of vertices that exist within the graph.
      return AdjMatrix.size();
    }


    int NumEdges() const {                                                                                              // This function returns the number of edges that exist within the graph.
      return numEdges;
    }


    bool addVertex(VertexT v) {                                                                                         // This function adds a vertex "v" to the graph, if there's space available.
        vector <EdgeData> EdgeDataVec;
        if (AdjMatrix.count(v) == 0) {                                                                                  // If we can add the vertex, we return true.
          AdjMatrix.emplace(v,EdgeDataVec);
          this->Vertices.push_back(v);
          return true;
        }

        return false;
    }


    bool addEdge(VertexT from, VertexT to, WeightT weight) {                                                            // This function adds an edge to the graph. If successful, it returns true.

        if (!isPresent(from, to)) {return false;}

        auto& edges = AdjMatrix.at(from);
        for (auto& edge : edges) {                                                                                      // Traversing through the edges, to verify the existence of "to" and "from" edges in our graph.
            if (edge.Vertex == to) {
                edge.Weight = weight;
                return true;
            }
        }

        EdgeData newEdge;                                                                                               // In case the edge doesn't exist, we allocate memory for "newEdge".
        newEdge.EdgeExists = true;
        newEdge.Vertex = to;
        newEdge.Weight = weight;

        edges.push_back(newEdge);
        numEdges++;                                                                                                     // Updating the "numEdges" attribute.
        return true;
    }


    bool getWeight(VertexT from, VertexT to, WeightT& weight) const {                                                   // This function calculates and returns the weight of edge between "from" and "to".

        if (!isPresent(from, to)) {return false;}

        for (auto& edge : AdjMatrix.at(from)) {
            if (edge.Vertex == to) {                                                                                    // Traversing the graph and setting the weight for the "edge".
              weight = edge.Weight;
              return true;
            }
        }

        return false;                                                                                                   // Return false, if no edge exists between "to" and "from".
    }

    //
    // neighbors
    //
    // Returns a set containing the neighbors of v, i.e. all
    // vertices that can be reached from v along one edge.
    // Since a set is returned, the neighbors are returned in
    // sorted order; use foreach to iterate through the set.
    //
    set<VertexT> neighbors(VertexT v) const {                                                                           // This function returns a set of neighbours to the vertex "v".

      if (!isPresentVertex(v)) {return set<VertexT>();}                                                                 // In case the vertex "v" doesn't exist in the graph, we return an empty set.

      set<VertexT>  neigh;
      for(auto &i : AdjMatrix.at(v)) {
          neigh.insert(i.Vertex);
      }

      return neigh;                                                                                                     // If the vertex exists, we return the set.
    }


    vector<VertexT> getVertices() const {
    return this->Vertices;  // returns a copy:
    }


    void dump(ostream& output) const {                                                                                  // This function outputs the state of our graph.

          output << "***************************************************" << endl;
          output << "********************* GRAPH ***********************" << endl;

          output << "**Num vertices: " << this->NumVertices() << endl;
          output << "**Num edges: " << this->NumEdges() << endl;
          output << endl;
          output << "**Vertices:" << endl;
          for (int i = 0; i < this->NumVertices(); i++) {
              output << " " << i << ". " << this->Vertices[i] << endl;

          }
          output << endl;
          output << "**Edges:" << endl;
          for(auto& i : AdjMatrix) {
              output << " " << i.first << ": ";
              for(auto& j : i.second)
              {
                  output << "("<< i.first <<","<< j.Vertex << "," << j.Weight << ") ";
              }
              output << endl;
          }
          output << "**************************************************" << endl;
        }
    };
