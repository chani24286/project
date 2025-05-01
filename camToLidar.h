#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Point3D {
    float x, y, z;
};

vector<Point3D> loadLidarData(const string& file_path) {
    ifstream file(file_path, ios::binary);
    vector<Point3D> points;
    if (!file) {
        cerr << "Failed to open file!" << endl;
        return points;
    }

    while (!file.eof()) {
        float x, y, z, intensity;
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
        if (file.gcount() < sizeof(float) * 4) break;
        points.push_back({ x, y, z });
    }
    return points;
}

float computeDistance(const vector<Point3D>& lidar_points,
    const Matrix4f& T,
    const Matrix3f& K,
    const tuple<int, int, int, int>& bbox) {
    vector<float> valid_depths;
    auto [x_min, y_min, x_max, y_max] = bbox;

    for (const auto& pt : lidar_points) {
        Vector4f lidar_pt(pt.x, pt.y, pt.z, 1.0f);
        Vector3f cam_pt = (T * lidar_pt).head<3>();

        if (cam_pt.z() <= 0) continue;

        Vector3f proj = K * cam_pt;
        float u = proj(0) / proj(2);
        float v = proj(1) / proj(2);

        if (u >= x_min && u <= x_max && v >= y_min && v <= y_max) {
            valid_depths.push_back(cam_pt.z());
        }
    }

    if (valid_depths.empty()) return -1.0f;

    float sum = 0;
    for (float d : valid_depths) sum += d;
    return sum / valid_depths.size();
}

float calculate(tuple<int, int, int, int> bbox, string lidarPath) {
    Matrix4f T;
    T << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
        1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
        9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
        0.0, 0.0, 0.0, 1.0;

    Matrix3f K;
    K << 984.2439, 0.0, 690.0,
        0.0, 980.8141, 233.1966,
        0.0, 0.0, 1.0;
    vector<Point3D> lidar_points = loadLidarData(lidarPath);
    float distance = computeDistance(lidar_points, T, K, bbox);

    if (distance > 0)
        return distance;
    else
        return -1.0;
}


