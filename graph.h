
#include <unordered_map>
#include <vector>


#pragma once
struct Node {
    long long id;
    double lat;
    double lon;
};

struct Edge {
    long long to;  // Neighboring node ID
    double distance;  // Distance to neighboring node
	std::string name; // street name
};
using Graph = std::unordered_map<long long, std::vector<Edge>>;
void remove_node(long long node);
void remove_edge(long long from, long long to);
void print_graph();
void parse_osm_file(const std::string& nodesfilename, const std::string& edgesfilename);
void add_node(long long id, double lat, double lon);
void add_edge(long long from, long long to, double distance, std::string name);

// דייקסטרה
std::unordered_map<long long, double> dijkstra( long long start, std::unordered_map<long long, long long>& previous);

// שחזור הנתיב
std::vector<Edge> reconstruct_path(long long start, long long target, const std::unordered_map<long long, long long>& previous);
long long findNearestNodeId(const std::unordered_map<long long, Node>& nodes, double lat, double lon);
std::vector<Edge> createGraph(std::string place);


