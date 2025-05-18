#pragma once
#include "Eigen/Eigen"
#include <vector>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>           
#include <pcl/point_types.h>         
#include <pcl/point_cloud.h>

#define _CRT_SECURE_NO_WARNINGS
#ifndef ICP_H
#define ICP_H
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;


using namespace std;


typedef struct {
    vector<float> distances;
    vector<int> indices;
} NEIGHBOR;


struct KDNode {
    int pointIndex; // index in the original matrix
    std::unique_ptr<KDNode> left;
    std::unique_ptr<KDNode> right;
};
struct ICP_out {
    Matrix4d T;
    int iter;
    double err;
};
int findNearestNeighbor(
    const MatrixXd& pts,          // 3ªN
    const KDNode* root,
    const Vector3d& query);
void nearestNeighborSearch(
    const MatrixXd& pts,          // 3ªN
    const KDNode* node,
    const Vector3d& query,
    int             depth,
    int& bestIdx,
    double& bestD2);
unique_ptr<KDNode> buildKDTree(
    const MatrixXd& pts,             // size 3ªN
    vector<int>& indices,
    int depth = 0);
ICP_out icp(const MatrixXd& A, const MatrixXd& B,  int max_iter, double tol);
Matrix4d best_fit_transform(const MatrixXd& A, const MatrixXd& B);
vector<int> findAllNearestNeighbors(
    const MatrixXd& pts,          // 3ªN
    const KDNode* root);
pcl::PointCloud<pcl::PointXYZ>::Ptr lidarReading(std::string path);
Matrix4d icpAlgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, Matrix4d T);


#endif