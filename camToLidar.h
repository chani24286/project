#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <Eigen/Dense>

struct Point3D {
    float x, y, z;
};

// הצהרות על מטריצות גלובליות
extern Eigen:: Matrix4f T;
extern Eigen::Matrix3f K;

// הצהרות על פונקציות
void initK();
void initT();
void initT(Eigen::Matrix4f new_t);
void initK(Eigen::Matrix3f new_k);
float rightDistance( float bbox[4]);
float leftDistance( float bbox[4]);
float AVGdistance( float bbox[4]);
float angle(float bbox[4]);
std::vector<Eigen::Vector3f> computeValidPoints( float bbox[4]);
std::vector<Point3D> loadLidarData(Eigen::MatrixXd& mat);
float maxDistance(float bbox[4]);