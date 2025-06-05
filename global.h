
#pragma once
#include <string>
#include <Eigen/Dense>
#include <unordered_map>
#include <atomic>

#define M_PI 3.14159265358979323846
constexpr double R = 6371000.0; // Radius of the Earth in meters
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
struct ICP_out {
   Eigen:: Matrix4d T;
    int iter;
    double err;
};
// מבצע תרגום של ענן נקודות בפורמט PCD לקובץ חדש
//void translatePointCloud(
//    const std::string& input_file,
//    const std::string& output_file,
//    float dx, float dy, float dz);

// ממיר מעלות לרדיאנים
double haversine(double lat1, double lon1, double lat2, double lon2);

// מחשב את המרחק בין שתי נקודות גאוגרפיות (lat/lon) בקילומטרים
double deg2rad(double degrees);
std::string exec(const char* cmd);

extern std::unordered_map<long long, std::vector<Edge>> graph;
extern std::unordered_map<long long, Node> nodes; 
extern std::atomic<bool> keepRunning;
void printing(std::string str);
void convertBinToPCD(std::string pathBin, std::string pathPCD);
Eigen::MatrixXd PCDtoMatrix(std::string path);
double metersToDegreesLat(double meters);
double metersToDegreesLon(double meters, double latitude_deg);
int transformGPS(std::string file_path, double dist);
Eigen::MatrixXd PCDtoMatrix(std::string path);