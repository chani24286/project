#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Point3D {
    float x, y, z;
};

// ������ �� ������� ��������
extern Matrix4f T;
extern Matrix3f K;

// ������ �� ��������
void initK();
void initT();
void initT(Matrix4f new_t);
void initK(Matrix3f new_k);
float rightDistance(const float bbox[4]);
float leftDistance(const float bbox[4]);
float AVGdistance(const float bbox[4]);
float angle(const float bbox[4]);
vector<Vector3f> computeValidPoints(const float bbox[4]);
vector<Point3D> loadLidarData(const string& file_path);
