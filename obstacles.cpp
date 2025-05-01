#include "obstacles.h"
#define _CRT_SECURE_NO_WARNINGS

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>
//#include <pcl/visualization/pcl_visualizer.h>


struct ClusterInfo {
    float y_min;   // ���� ��� ����� ������ (��� Y)
    float y_max;   // ���� ��� ���� ������ (��� Y)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // ������ ����
};

float findPassage(float y_min_limit, float y_max_limit, float dist)
{
    // ���� �� ��� �������
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file input.pcd \n");
        return (-1);
    }
    std::cout << "Number of points in cloud: " << cloud->points.size() << std::endl;


    // ��� 1: ����� ��� ���� Z ������� X
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    // ����� ������ ������ ��� �� ������ ���
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.05, 2.0); // 5 �"� �� 2 ��� ����
    pass.filter(*cloud);
    

    // ����� ������ ������ ���
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, dist); 
    pass.filter(*cloud);
    std::cout << "Number of points after filtering: " << cloud->points.size() << std::endl;
    // ��� 2: ����� KDTree ��������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // ��� 3: ����� �������
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3); // ���� ������� ��� ������ ��� ������ ����� �����
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // ��� 4: ����� ���� ��������
    std::vector<ClusterInfo> left_clusters;
    std::vector<ClusterInfo> right_clusters;

    for (const auto& indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        float y_min = std::numeric_limits<float>::max();
        float y_max = std::numeric_limits<float>::lowest();

        for (int idx : indices.indices)
        {
            cluster->points.push_back(cloud->points[idx]);
            float y = cloud->points[idx].y;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }

        ClusterInfo info{ y_min, y_max, cluster };

        if (y_max > 0)
            left_clusters.push_back(info);   // �� ����
        else
            right_clusters.push_back(info);  // �� ����
    }
    // ����� ������ ���� ���� ���� ����� �����
    ClusterInfo right_dummy;
    right_dummy.y_min = -std::numeric_limits<float>::infinity();
    right_dummy.y_max = y_min_limit;
    right_dummy.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    right_clusters.push_back(right_dummy);

    // ����� ������ ���� ���� ���� ������ �����
    ClusterInfo left_dummy;
    left_dummy.y_min = y_max_limit;
    left_dummy.y_max = std::numeric_limits<float>::infinity();
    left_dummy.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    left_clusters.push_back(left_dummy);


    // ��� 5: ���� �������� ��� �����
    std::sort(left_clusters.begin(), left_clusters.end(), [](const ClusterInfo& a, const ClusterInfo& b) {
        return a.y_max > b.y_max; // ����� ����� ��� ������
        });

    std::sort(right_clusters.begin(), right_clusters.end(), [](const ClusterInfo& a, const ClusterInfo& b) {
        return a.y_min < b.y_min; // ����� ����� ��� �����
        });

    // ��� 6: ����� ���� ����
    bool found_passage = false;
    float window_size = 0.06; // ���� �� 60 �"� ������
    for (const auto& cluster : left_clusters)
        std::cout << "Left cluster: y_min=" << cluster.y_min << ", y_max=" << cluster.y_max << std::endl;

    for (const auto& cluster : right_clusters)
        std::cout << "Right cluster: y_min=" << cluster.y_min << ", y_max=" << cluster.y_max << std::endl;

    float gap_middle = right_clusters[0].y_min - left_clusters.back().y_max;
    if (gap_middle >= window_size) {
        std::cout << "Found passage between left and right clusters continue straight: " << gap_middle << std::endl;
        return 0.0;
    }
    // ����� ��� ������� ��� ����
    for (size_t i = 0; i + 1 < left_clusters.size(); ++i)
    {
        float gap = left_clusters[i].y_min - left_clusters[i + 1].y_max;
        if (gap >= window_size)
        {
            std::cout << "Found passage in left: " <<left_clusters[i].y_min << " meters\n";
            found_passage = true;
			return left_clusters[i].y_min;
        }
    }

    // ����� ��� ������� ��� ����
    if (!found_passage)
    {
        for (size_t i = 0; i + 1 < right_clusters.size(); ++i)
        {
            float gap = right_clusters[i + 1].y_min - right_clusters[i].y_max;
            if (gap >= window_size)
            {
                std::cout << "Found passage in right: " << right_clusters[i].y_max << " meters from you\n";
                found_passage = true;
				return right_clusters[i].y_max;
            }
        }
    }

    if (!found_passage)
        std::cout << "No passage found\n";

    return -1.1;
}
int instructionsObs(float x, float y, float distance) {
	float space= findPassage(x,y, distance);
    std::string direction;
	if (space == -1.1) {
		std::cout << "No passage found\n";
		return -1;
	}
    if (space == 0) {
		direction = "continue straight";
    }
	else if (space > 0) {
        direction = " left";
	}
	else {
		
        direction = " right";
	}
    std::string command = "python \"C:\\Users\\User\\Documents\\projectC\\tts.py\"" + direction;

    int result = system(command.c_str());

    if (result != 0) {
        std::cerr << "Error running Python script!" << std::endl;
    }

}
