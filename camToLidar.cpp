#include "camToLidar.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#define M_PI 3.14159265358979323846
using namespace std;
using namespace Eigen;

Matrix4f T;
Matrix3f K;

void initT(Matrix4f new_t) {
    T = new_t;
}
void initK(Matrix3f new_k) {
    K = new_k;
}

void initT() {
    T << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
        1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
        9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
        0.0, 0.0, 0.0, 1.0;
}
void initK() {
    K << 984.2439, 0.0, 690.0,
        0.0, 980.8141, 233.1966,
        0.0, 0.0, 1.0;
}
vector<Point3D> loadLidarData(const string& file_path) {
    ifstream file(file_path);
    vector<Point3D> points;
    string line;

    if (!file) {
        cerr << "Failed to open file!" << endl;
        return points;
    }

    while (getline(file, line)) {
        stringstream ss(line);
        float x, y, z;
        if (ss >> x >> y >> z) {  // אם השורה מכילה 3 ערכים (x, y, z)
            points.push_back({ x, y, z });
        }
    }
    std::cout << "Loaded " << points.size() << " points from " << file_path << endl;
    return points;
}

vector<Vector3f> computeValidPoints(const float bbox[4]) {
    float x_min = bbox[0], y_min = bbox[1], x_max = bbox[2], y_max = bbox[3];
    vector<Vector3f> in_bbox_points;
    const vector<Point3D>& lidar_points = loadLidarData("C:\\Users\\User\\Downloads\\0000000371.txt");

    for (const auto& pt : lidar_points) {
        Vector4f lidar_pt(pt.x, pt.y, pt.z, 1.0f);
        Vector3f cam_pt = (T * lidar_pt).head<3>();

        if (cam_pt.z() <= 0) continue;

        Vector3f proj = K * cam_pt;
        float u = proj(0) / proj(2);
        float v = proj(1) / proj(2);

        if (u >= x_min && u <= x_max && v >= y_min && v <= y_max) {
            in_bbox_points.push_back(cam_pt);
        }
    }
    return in_bbox_points;
}

float angle(const float bbox[4]) {
    const vector<Vector3f> in_bbox_points = computeValidPoints(bbox);
    // נחשב את הזווית של מרכז האובייקט (ניקח את הממוצע)
    Vector3f avg_point(0, 0, 0);
    for (const auto& pt : in_bbox_points)
        avg_point += pt;
    avg_point /= in_bbox_points.size();

    float angle_rad = atan2(avg_point.x(), avg_point.z()); // זווית יחסית לציר z
    float angle_deg = angle_rad * 180.0f / M_PI; // המרה למעלות
    return angle_deg;
}
float AVGdistance(const float bbox[4]) {
    const vector<Vector3f> valid_depths = computeValidPoints(bbox);
    if (valid_depths.empty()) return -1.0f;
    float sum = 0;
    for (const auto& d : valid_depths) sum += d.z();
    return sum / valid_depths.size();
}
float leftDistance( float bbox[4]) {
    float margin = 20.0f;
    bbox[1] -= margin;
	bbox[3] += margin;
	return AVGdistance(bbox);
}
float rightDistance( float bbox[4]) {
    float margin = 20.0f;
    bbox[2] -= margin;
    bbox[3] += margin;
    return AVGdistance(bbox);
}
float maxDistance(float bbox[4]) {
    const vector<Vector3f> valid_depths = computeValidPoints(bbox);
    if (valid_depths.empty()) return -1.0f;
    float max = 0;
    for (const auto& d : valid_depths) max = (max < d.z) ? d.z;
    return max;
}
