
#pragma once
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include "graph.h"
#include <limits>
#include <queue>
#include <algorithm>
#include "global.h"
#include <cstdio>
#include "NevigationObject.h"


const double M_PI = 3.141592653589793;
constexpr double R = 6371e3; // Radius of the Earth in meters
// Graph: node ID -> list of edges
std::unordered_map<long long, std::vector<Edge>> graph;
std::unordered_map<long long, Node> nodes; // node ID -> node data


// Convert degrees to radians
//double deg2rad(double deg) {
//    return deg * M_PI / 180.0;
//}
//
//// Calculate distance using Haversine formula
//double haversine(double lat1, double lon1, double lat2, double lon2) {
//    double dlat = deg2rad(lat2 - lat1);
//    double dlon = deg2rad(lon2 - lon1);
//    double a = sin(dlat / 2) * sin(dlat / 2) +
//        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
//        sin(dlon / 2) * sin(dlon / 2);
//    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
//    return R * c;
}
//find my coordinates by google maps request:
//double calculate_distance_osrm(double lat1, double lon1, double lat2, double lon2) {
//	std::string command="python\"C:\\Users\\User\\Documents\\projectC\\trail.py""
//    double result = system(command.c_str());
//    return double;
//}

// Add a node to the graph
void add_node(long long id, double lat, double lon) {
    nodes[id] = { id, lat, lon };
    graph[id]; // create entry if doesn't exist
}

// Add an undirected edge (two directions)
void add_edge(long long from, long long to, double distance, std::string name) {
    // Add edge from 'from' to 'to'
    graph[from].push_back(Edge{ to, distance, name });

    // Add edge from 'to' to 'from'
    graph[to].push_back(Edge{ from, distance, name });
}

void remove_node(long long node) {
    for (const Edge& e : graph[node]) {
        auto& neighbor_edges = graph[e.to];
        neighbor_edges.erase(
            std::remove_if(neighbor_edges.begin(), neighbor_edges.end(),
                [node](const Edge& edge) { return edge.to == node; }),
            neighbor_edges.end()
        );
    }
    // מחיקה מהגרף
    graph.erase(node);
}



// Remove an undirected edge (both directions)
void remove_edge(long long from, long long to) {
    auto& from_edges = graph[from];
    from_edges.erase(std::remove_if(from_edges.begin(), from_edges.end(),
        [to](const Edge& edge) { return edge.to == to; }), from_edges.end());

    auto& to_edges = graph[to];
    to_edges.erase(std::remove_if(to_edges.begin(), to_edges.end(),
        [from](const Edge& edge) { return edge.to == from; }), to_edges.end());
}





// Print the graph
void print_graph() {
    std::cout << "Graph contains " << graph.size() << " nodes.\n";
    for (const auto& pair : graph) {
        long long id = pair.first;
        const auto& edges = pair.second;

        std::cout << "Node " << id << " connects to: ";
        for (const auto& edge : edges) {
            std::cout << edge.name << " (" << edge.distance << "m), ";
        }
        std::cout << "\n";
    }
}


void parse_osm_file(const std::string& nodesfilename, const std::string& edgesfilename) {
    nlohmann::json data;
    std::ifstream(nodesfilename) >> data;
    for (const auto& element : data["features"]) {
        long long id = element["properties"]["osmid"];
        double lat = element["properties"]["y"];
        double lon = element["properties"]["x"];
        add_node(id, lat, lon);
        add_node(id * -1, lat, lon);

        if (element["properties"]["highway"].contains("crossing")) 
            add_edge(element["id"], element["id"] * -1, 2.0, "traffic signals");
		if (element["properties"]["highway"].contains("traffic_light"))
			add_edge(element["id"], element["id"] * -1, 2.0, "crossing");

    }
    std::ifstream(edgesfilename) >> data;
    for (const auto& element : data["features"]) {

        long long from = element["properties"]["u"];
        long long to = element["properties"]["v"];
        double distance = element["properties"]["length"];
		std::string name = element["properties"]["name"];
        add_edge(from, to, distance, name);
        add_edge(from * -1, to * -1, distance, name);

    }
    
}



std::unordered_map<long long, double> dijkstra( long long start, std::unordered_map<long long, long long>& previous) {
    std::unordered_map<long long, double> distances;
    previous.clear();

    for (const auto& pair : graph) {
        distances[pair.first] = std::numeric_limits<double>::infinity();
    }
    distances[start] = 0.0;

    using P = std::pair<double, long long>; // (distance, node ID)
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.emplace(0.0, start);

    while (!pq.empty()) {
        P top = pq.top();
        double dist = top.first;
        long long u = top.second;
        pq.pop();

        if (dist > distances[u]) continue; // Skip outdated entry

        for (const auto& edge : graph.at(u)) {
            long long v = edge.to;
            double weight = edge.distance;
            double alt = dist + weight;
            if (alt < distances[v]) {
                distances[v] = alt;
                previous[v] = u;
                pq.emplace(alt, v);
            }
        }
    }

    return distances;
}
const Edge* get_edge( long long from, long long to) {
    auto it = graph.find(from);

    for (const Edge& edge : it->second) {
		if (edge.to == to) return &edge;
      }
	return nullptr; // Edge not found
}
std::vector<Edge> reconstruct_path(long long start, long long target, const std::unordered_map<long long, long long>& previous) {
    std::vector<Edge> path;
	//first - where i am comming from, second- where i am going to
    long long at = target;
    auto it=previous.find(at);;
    while (at != start) {
		 const Edge* edge = get_edge( at, it->first);
        path.push_back(*edge);
        
        if (it == previous.end()) return {}; // no path
        at = it->second;
        it = previous.find(at);
    }
	const Edge* edge = get_edge( start, it->first);
    path.push_back(*edge);
    std::reverse(path.begin(), path.end());
    return path;
}
long long findNearestNodeId(const std::unordered_map<long long, Node>& nodes,  double lat,double lon ) {
    long long nearestId = -1;
    double minDist = std::numeric_limits<double>::max();

    for (const auto& [id, node] : nodes) {
		double dist = haversine(lat, lon, node.lat, node.lon);
        if (dist < minDist) {
            minDist = dist;
            nearestId = id;
        }
    }

    return nearestId;
}
 createGraph(std::string place ,NevigationObject &me) {


    std::string path = "python\"C:\\Users\\User\\Documents\\projectC\\Map.py"+place+"\"";
	system(path.c_str());
    std::string nodesJSON = "C:\\Users\\User\\Documents\\projectC\\nodes.geojson";
    std::string edgesJSON  = "C:\\Users\\User\\Documents\\projectC\\edges.geojson";

    // Parse the OSM file and build the graph
    parse_osm_file(nodesJSON, edgesJSON);

    // Print the graph
  //  print_graph();
    //find my node id
    path="python\"C:\\Users\\User\\Documents\\rojectC\\myCoordinates.py\"";
    std::string result = exec(path.c_str());
    double lat, lon;
    sscanf(result.c_str(), "%lf,%lf", &lat, &lon);
    long long start = findNearestNodeId(nodes, lat, lon);
    //finds the target node id
    path = "python\"C:\\Users\\User\\Documents\\rojectC\\placeToCoordinates.py\""+place;
    result = exec(path.c_str());
    sscanf(result.c_str(), "%lf,%lf", &lat, &lon);
    long long target = findNearestNodeId(nodes, lat, lon); 

    std::unordered_map<long long, long long> previous;
    auto distances = dijkstra( start, previous);
    std::vector<Edge> allpath = reconstruct_path(start, target, previous);
    me.setTrail(allpath);

}
