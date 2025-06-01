#pragma once
#include <vector>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>           
#include <pcl/point_types.h>         
#include <pcl/point_cloud.h>
#include "global.h"
#include <memory>
#include <iostream>
#include <string>
#define _CRT_SECURE_NO_WARNINGS

typedef struct {
   std:: vector<float> distances;
   std:: vector<int> indices;
} NEIGHBOR;


struct KDNode {
    int pointIndex; // index in the original matrix
    std::unique_ptr<KDNode> left;
    std::unique_ptr<KDNode> right;
};

int findNearestNeighbor(
    const Eigen::MatrixXd& pts,          // 3ªN
    const KDNode* root,
    const Eigen::Vector3d& query);
void nearestNeighborSearch(
    const Eigen::MatrixXd& pts,          // 3ªN
    const KDNode* node,
    const Eigen::Vector3d& query,
    int             depth,
    int& bestIdx,
    double& bestD2);
std::unique_ptr<KDNode> buildKDTree(
    const Eigen::MatrixXd& pts,             // size 3ªN
   std::vector<int>& indices,
    int depth = 0);
ICP_out icp(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,  int max_iter, double tol);
Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
std::vector<int> findAllNearestNeighbors(
    const Eigen::MatrixXd& pts,          // 3ªN
    const KDNode* root);
pcl::PointCloud<pcl::PointXYZ>::Ptr lidarReading(std::string path);
ICP_out icpAlgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
    bool movements(ICP_out out);
    Eigen::MatrixXd convertPCLtoMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
