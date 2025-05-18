#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <numeric>
#include <vector>
#include <memory>
#include <limits>
#include "icp.h"
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>           
#include <pcl/point_types.h>         
#include <pcl/point_cloud.h>         
#include "sensors.h"

using namespace std;
using namespace Eigen;
unique_ptr<KDNode> buildKDTree(
    const MatrixXd& pts,             // size 3ªN
    vector<int>& indices,
    int depth)
{
    if (indices.empty()) return nullptr;
    int axis = depth % 3;
    int mid = indices.size() / 2;
    // Partition so that indices[mid] is the median by pts(axis, *)
    nth_element(
        indices.begin(), indices.begin() + mid, indices.end(),
        [&](int a, int b) { return pts(axis, a) < pts(axis, b); });

    auto node = make_unique<KDNode>();
    node->pointIndex = indices[mid];
    // left: [0..mid), right: (mid..end)
    vector<int> leftIdx(indices.begin(), indices.begin() + mid);
    vector<int> rightIdx(indices.begin() + mid + 1, indices.end());
    node->left = buildKDTree(pts, leftIdx, depth + 1);
    node->right = buildKDTree(pts, rightIdx, depth + 1);
    return node;
}
// Recursive nearest neighbor search in 3D
void nearestNeighborSearch(
    const MatrixXd& pts,          // 3ªN
    const KDNode* node,
    const Vector3d& query,
    int             depth,
    int& bestIdx,
    double& bestD2)
{
    if (!node) return;
    int axis = depth % 3;
    int idx = node->pointIndex;

    // check this node
    double d2 = (pts.col(idx) - query).squaredNorm();
    if (d2 < bestD2) {
        bestD2 = d2;
        bestIdx = idx;
    }
    // choose which side to visit first
    bool goLeft = query(axis) < pts(axis, idx);
    auto* first = goLeft ? node->left.get() : node->right.get();
    auto* second = goLeft ? node->right.get() : node->left.get();

    // search near side
    nearestNeighborSearch(pts, first, query, depth + 1, bestIdx, bestD2);

    // decide if far side could contain closer point
    double diff = query(axis) - pts(axis, idx);
    if (diff * diff < bestD2) {
        nearestNeighborSearch(pts, second, query, depth + 1, bestIdx, bestD2);
    }
}

// Find nearest neighbor index for a single query
int findNearestNeighbor(
    const MatrixXd& pts,          // 3ªN
    const KDNode* root,
    const Vector3d& query)
{
    int bestIdx = -1;
    double bestD2 = numeric_limits<double>::infinity();
    nearestNeighborSearch(pts, root, query, 0, bestIdx, bestD2);
    return bestIdx;
}
// Find nearest neighbor for each column (point) in pts
vector<int> findAllNearestNeighbors(
    const MatrixXd& pts,          // 3ªN
    const KDNode* root)
{
    int N = pts.cols();
    vector<int> nbrs(N);
    for (int i = 0; i < N; ++i) {
        nbrs[i] = findNearestNeighbor(pts, root, pts.col(i));
    }
    return nbrs;
}

//ICP using KD Tree
Matrix4d best_fit_transform(const MatrixXd& A, const MatrixXd& B) {
    int N = A.rows();
    // compute centroids
    Vector3d cA = Vector3d::Zero(), cB = Vector3d::Zero();
    for (int i = 0; i < N; i++) {
        cA += A.row(i).transpose();
        cB += B.row(i).transpose();
    }
    cA /= N;  cB /= N;

    // demean
    MatrixXd AA = A, BB = B;
    for (int i = 0; i < N; i++) {
        AA.row(i) -= cA.transpose();
        BB.row(i) -= cB.transpose();
    }
    // compute SVD of H = AA BB
    Matrix3d H = AA.transpose() * BB;
    JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
    Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0)
        R.col(2) *= -1;

    Vector3d t = cB - R * cA;
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}
ICP_out icp(const MatrixXd& A, const MatrixXd& B,int max_iter, double tol)
{
    int N = A.rows();
    // prepare homogeneous src
    MatrixXd srcH(4, N), src3d(3, N), dstH(4, N), dst3d(3, N);
    srcH.setOnes(); dstH.setOnes();
    for (int i = 0; i < N; i++) {
        srcH.block<3, 1>(0, i) = A.row(i).transpose();
        src3d.col(i) = A.row(i).transpose();
        dstH.block<3, 1>(0, i) = B.row(i).transpose();
    }
    // build KD tree on target points B
    MatrixXd B3 = B.transpose();     // 3ªN
    vector<int> idxs(N);
    iota(idxs.begin(), idxs.end(), 0);
    auto tree = buildKDTree(B3, idxs, 0);

    Matrix4d T = Matrix4d::Identity();
    double prevErr = 0;
    int iterUsed = 0;
    for (int iter = 0; iter < max_iter; ++iter) {
        // find NN for each transformed source point
        auto nn = findAllNearestNeighbors(B3, tree.get());

        // reassemble dst3d in correspondence order
        for (int i = 0; i < N; i++)
            dst3d.col(i) = B3.col(nn[i]);

        // estimate new transform from original A to dst3d
        T = best_fit_transform(A, dst3d.transpose());

        // apply to srcH & update src3d
        srcH = T * srcH;
        for (int i = 0; i < N; i++)
            src3d.col(i) = srcH.block<3, 1>(0, i);

        // compute mean error
        double err = 0;
        for (int i = 0; i < N; i++)
            err += (src3d.col(i) - dst3d.col(i)).squaredNorm();
        err = sqrt(err / N);

        if (iter > 0 && fabs(prevErr - err) < tol) {
            iterUsed = iter;
            break;
        }
        prevErr = err;
    }

    return { T , iterUsed, prevErr};
}
pcl::PointCloud<pcl::PointXYZ>::Ptr lidarReading(std::string path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(path, *cloud2);
    return cloud2;
}
ICP_out icpAlgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2){
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //   cloud = lidar.getCurrent_scan();
   
    Eigen::MatrixXd A(cloud->points.size(), 3);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        A(i, 0) = cloud->points[i].x;
        A(i, 1) = cloud->points[i].y;
        A(i, 2) = cloud->points[i].z;
    }
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  //  pcl::io::loadPCDFile(path, *cloud2);
    //lidar.setCurrent_scan(cloud2);
    Eigen::MatrixXd B(cloud2->points.size(), 3);
    for (size_t i = 0; i < cloud2->points.size(); ++i) {
        B(i, 0) = cloud2->points[i].x;
        B(i, 1) = cloud2->points[i].y;
        B(i, 2) = cloud2->points[i].z;
    }
    ICP_out out=  icp(A, B, 50, 0.001);
   

    return out;
}
bool movements(ICP_out out) {
    return out.err > 0.05 || out.iter > 30;
}
