#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "global.h"
#include <mutex>
#include <atomic>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <iomanip>


std::mutex print_mutex;
std::unordered_map<long long, std::vector<Edge>> graph;
std::unordered_map<long long, Node> nodes;
std::atomic<bool> keepRunning(true);
//void translatePointCloud(
//    const std::string& input_file,
//    const std::string& output_file,
//    float dx, float dy, float dz)
//{
//    // קריאה מהקובץ
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
//    {
//        PCL_ERROR("Couldn't read the input file \n");
//        return;
//    }
//
//    // מטריצת טרנספורמציה: זהות + הזזה
//    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//    transform.translation() << dx, dy, dz;
//
//    // החלת הטרנספורמציה
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
//
//    // שמירה לקובץ
//    pcl::io::savePCDFileBinary(output_file, *transformed_cloud);
//}
double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
        sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd, "r"), _pclose);
    if (!pipe) {
        throw std::runtime_error("popen failed");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
void printing(std::string str) {
    std::lock_guard<std::mutex> lock(print_mutex);
    std::cout << str << std::endl;
}


void convertBinToPCD(std::string pathBin, std::string pathPCD)
{
    struct Point { float x, y, z; };
    std::ifstream input(pathBin, std::ios::binary);
    if (!input) {
        std::cerr << "Failed to open input.bin\n";
    }
    std::vector<Point> pts;
    while (true) {
        float buf[4];
        input.read(reinterpret_cast<char*>(buf), sizeof(buf));
        if (input.gcount() < sizeof(buf)) break;
        pts.push_back({ buf[0], buf[1], buf[2] });
    }
    input.close();

    std::ofstream out(pathPCD);
    out << "# .PCD v0.7 - Point Cloud Data file format\n";
    out << "VERSION 0.7\n";
    out << "FIELDS x y z\n";
    out << "SIZE 4 4 4\n";
    out << "TYPE F F F\n";
    out << "COUNT 1 1 1\n";
    out << "WIDTH " << pts.size() << "\n";
    out << "HEIGHT 1\n";
    out << "VIEWPOINT 0 0 0 1 0 0 0\n";
    out << "POINTS " << pts.size() << "\n";
    out << "DATA ascii\n";
	// Write points to the PCD file
    for (auto& p : pts) {
        out << p.x << " " << p.y << " " << p.z << "\n";
    }
    out.close();
}
Eigen::MatrixXd PCDtoMatrix(std::string path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(path, *cloud);
    Eigen::MatrixXd A(cloud->points.size(), 3);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        A(i, 0) = cloud->points[i].x;
        A(i, 1) = cloud->points[i].y;
        A(i, 2) = cloud->points[i].z;
    }
    return A;
}
double metersToDegreesLat(double meters) {
    return meters / 111320.0;
}

double metersToDegreesLon(double meters, double latitude_deg) {
    return meters / (111320.0 * std::cos(latitude_deg * M_PI / 180.0));
}
int transformGPS(std::string file_path, double dist) {
    //"C:\\Users\\User\\Documents\\projectC\\data\\GPS.txt";  

    // קריאה
    std::ifstream in(file_path);
    if (!in) {
		std::cerr << "err in opening file: " << file_path << "\n";
        return 1;
    }

    double lat, lon;
    in >> lat >> lon;
    in.close();

    double delta_lat = metersToDegreesLat(dist);
    double delta_lon = metersToDegreesLon(dist, lat);

    lat += delta_lat;
    lon += delta_lon;
	//write new values to the file - override the old values
    std::ofstream out(file_path, std::ios::trunc); // trunc מוחק את תוכן הקובץ
    if (!out) {
        std::cerr << "err by opening file to write.\n";
        return 1;
    }
    out << std::fixed << std::setprecision(10); 
    out << lat << " " << lon << "\n";
    out << lat << " " << lon << "\n";
    out.close();

    return 0;
}