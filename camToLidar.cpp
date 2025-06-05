#define _CRT_SECURE_NO_WARNINGS

#include "camToLidar.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "global.h"
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
vector<Point3D> loadLidarData(Eigen::MatrixXd& mat) {
    std::vector<Point3D> points;
    points.reserve(mat.rows());

    for (int i = 0; i < mat.rows(); ++i) {
        Point3D p;
        p.x = static_cast<float>(mat(i, 0));
        p.y = static_cast<float>(mat(i, 1));
        p.z = static_cast<float>(mat(i, 2));
        points.push_back(p);
    }

    return points;
}

vector<Vector3f> computeValidPoints(float bbox[4]) {
    initK();
	initT();
    float x_min = bbox[0], y_min = bbox[1], x_max = bbox[2], y_max = bbox[3];
    vector<Vector3f> in_bbox_points;
    Eigen::MatrixXd mat=PCDtoMatrix("C:\\Users\\User\\Documents\\projectC\\data\\lidar1.pcd");
    vector<Point3D> lidar_points = loadLidarData(mat);

    for (const auto& pt : lidar_points) {
        Vector4f lidar_pt(pt.x, pt.y, pt.z, 1.0f);
        Vector3f cam_pt = (T * lidar_pt).head<3>();

        if (cam_pt.z() <= 0) continue;

        Vector3f proj = K * cam_pt;
  //      std::cout<<"k: " << K.transpose() << endl;
  //      std::cout << "T: " << T.transpose() << endl;
  //      std::cout << "cam_pt: " << cam_pt.transpose() << endl;
		//std::cout << proj(0) << " " << proj(1) << " " << proj(2) << std::endl;
        float u = proj(0) / proj(2);
        float v = proj(1) / proj(2);

        if (u >= x_min && u <= x_max && v >= y_min && v <= y_max) {
            in_bbox_points.push_back(cam_pt);
        }
    }
    return in_bbox_points;
}

float angle(float bbox[4]) {
    const vector<Vector3f> in_bbox_points = computeValidPoints(bbox);
    //computing average angle, by getting all the relevant sidtances
    Vector3f avg_point(0, 0, 0);
    for (const auto& pt : in_bbox_points)
        avg_point += pt;
    avg_point /= in_bbox_points.size();

    float angle_rad = atan2(avg_point.x(), avg_point.z()); // זווית יחסית לציר z
    float angle_deg = angle_rad * 180.0f / M_PI; // המרה למעלות
    return angle_deg;
}
float AVGdistance(float bbox[4]) {
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
    for (const auto& d : valid_depths) max = (max < d.z()) ? d.z() : max;
    return max;
}
