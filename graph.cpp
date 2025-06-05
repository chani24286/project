#define _CRT_SECURE_NO_WARNINGS

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


double calc_angle(double a, double b, double c) {

    double numerator = a * a + b * b - c * c;
    double denominator = 2 * a * b;

    // למנוע שגיאות דיוק נקודה צפה קטנות שעלולות לגרום לארגומנט להיות מחוץ לטווח [-1, 1]
    double cos_C = numerator / denominator;
    if (cos_C > 1.0) cos_C = 1.0;
    if (cos_C < -1.0) cos_C = -1.0;

    // חישוב הזווית ברדיאנים
    return std::acos(cos_C);
}
//find my coordinates by google maps request:
//double calculate_distance_osrm(double lat1, double lon1, double lat2, double lon2) {
//	std::string command="python\"C:\\Users\\User\\Documents\\projectC\\trail.py""
//    double result = system(command.c_str());
//    return double;
//}
double calculate_angle(double lat1, double lon1, double lat2, double lon2, double lat3, double lon3) {
    double a = haversine(lat1, lon1, lat2, lon2);
    double b = haversine(lat2, lon2, lat3, lon3);
    double c = haversine(lat3, lon3, lat1, lon1);
    double angle = calc_angle(a, b, c);
    return angle;
}

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
        try {
            long long id = element["properties"]["osmid"].get<long long>();
            double lat = element["properties"]["y"].get<double>();
            double lon = element["properties"]["x"].get<double>();
            add_node(id, lat, lon);
            add_node(id * -1, lat, lon);
            if (element["properties"]["highway"].is_string()) {
                std::string highway_str = element["properties"]["highway"].get<std::string>(); // חלץ את המחרוזת
                if (highway_str.find("crossing") != std::string::npos) 
                    add_edge(id, id * -1, 2.0, "crossing");
                if (highway_str.find("traffic_light") == std::string::npos)
                    add_edge(id, id * -1, 2.0, "traffic signals");
                
            }
        }
        catch (const nlohmann::json::exception& e) {
            std::cerr << "Warning: Missing or invalid essential node properties. Error: " << e.what() << ". Skipping element." << std::endl;
            continue;
        }
        for (auto const& [u, neighbors] : graph) {
            for (auto const& edge : neighbors) {
                if (!graph.count(edge.to)) {
                    graph[edge.to] = {}; 
                }
            }
        }

    }
    std::ifstream(edgesfilename) >> data;
    for (const auto& element : data["features"]) {

        long long from = element["properties"]["u"].get<long long>();
        long long to = element["properties"]["v"].get<long long>();
        double distance = element["properties"]["length"].get<double>();
		std::string name = (element["properties"]["name"].is_string())?element["properties"]["name"]:"unknown name";
        add_edge(from, to, distance, name);
        add_edge(from * -1, to * -1, distance, name);
    } 
}



std::unordered_map<long long, double> dijkstra( long long start, std::unordered_map<long long, long long>& previous) {
    std::unordered_map<long long, double> distances;
    previous.clear();
    for (const auto& pair : graph) {
        distances[pair.first];
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
        auto it = graph.find(u);
        if (it == graph.end()) {
            // אין שום רשימת שכנים עבור u, נמשיך הלאה
            continue;
        }

        for (const auto& edge : it->second) {
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
    long long at = target;

    while (at != start) {
        auto it = previous.find(at);
        if (it == previous.end()) {
            // אין מסלול אל היעד
            return {};
        }
        long long from = it->second;
        const Edge* edge = get_edge(from, at);  // מחפשים את הקשת מ־from אל at
        if (!edge) {
            return {};
        }
        path.push_back(*edge);
        at = from; 
    }

    std::reverse(path.begin(), path.end());
    return path;
}

    long long findNearestNodeId(const std::unordered_map<long long, Node>& nodes,  double lat,double lon ) {
	std::cout << "Finding nearest node to coordinates: (" << lat << ", " << lon << ")\n";
    long long nearestId = -1;
    double minDist = std::numeric_limits<double>::max();

    for (const auto& [id, node] : nodes) {
		double dist = haversine(lat, lon, node.lat, node.lon);
        if (dist < minDist) {
            minDist = dist;
            nearestId = id;
        }
    }
	std::cout << "Nearest node ID: " << nearestId << " with distance: " << minDist << "\n";
    return nearestId;
}
    void createGraph(std::string place ,NevigationObject& me) {

        std::string script_path = "C:\\Users\\User\\Documents\\projectC\\Map.py";
        std::string argument = "Karmiel, Israel";

        std::string path = "python \"" + script_path + "\" \"" + argument + "\"";
        std::cout << "Executing command: " << path << std::endl;
	    system(path.c_str());
         std::string nodesJSON = "C:\\Users\\User\\Documents\\projectC\\nodes.geojson";
        std::string edgesJSON  = "C:\\Users\\User\\Documents\\projectC\\edges.geojson";

    // Parse the OSM file and build the graph
    parse_osm_file(nodesJSON, edgesJSON);

    // Print the graph
  //  print_graph();
    //find my node id
   path="python \"C:\\Users\\User\\Documents\\projectC\\myCoordinates.py\"";
    std::string result = exec(path.c_str());
    double lat, lon;
    sscanf_s(result.c_str(), "%lf,%lf", &lat, &lon);
    long long start = findNearestNodeId(nodes, lat, lon);
    //finds the target node id
    place = "Zahal 104 Karmiel, Israel";
    path = "python \"C:\\Users\\User\\Documents\\projectC\\placeToCoordinates.py\" \""+place+"\"";
    result = exec(path.c_str());
    sscanf_s(result.c_str(), "%lf,%lf", &lat, &lon);
    long long target = findNearestNodeId(nodes, lat, lon); 

    std::unordered_map<long long, long long> previous;
    auto distances = dijkstra( start, previous);
    std::vector<Edge> allpath = reconstruct_path(start, target, previous);
    me.setTrail(allpath);
    me.setLastNode(nodes[start]);
}
