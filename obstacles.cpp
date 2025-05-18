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
#include "NevigationObject.h"
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float dist) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    // סינון נקודות נמוכות מדי או גבוהות מדי
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.05, 2.0); // 5 ס"מ עד 2 מטר גובה
    pass.filter(*cloud);


    // סינון נקודות רחוקות מדי
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, dist);
    pass.filter(*cloud);
    return cloud;
}

float findPassage(float y_min_limit, float y_max_limit, float dist, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, NevigationObject &me)
{
 

    // שלב 1: סינון לפי גובה Z ורחוקות X
    cloud = filter(cloud, dist);
    // שלב 2: יצירת KDTree לאשכולות
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // שלב 3: חיפוש אשכולות
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3); // מרחק מקסימלי בין נקודות כדי להיחשב באותו אשכול
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // שלב 4: החזקת פרטי האשכולות
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
            left_clusters.push_back(info);   // צד שמאל
        else
            right_clusters.push_back(info);  // צד ימין
    }
    // אשכול שמייצג גבול קבוע מהצד הימני ביותר
    ClusterInfo right_dummy;
    right_dummy.y_min = -std::numeric_limits<float>::infinity();
    right_dummy.y_max = y_min_limit;
    right_dummy.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    right_clusters.push_back(right_dummy);

    // אשכול שמייצג גבול קבוע מהצד השמאלי ביותר
    ClusterInfo left_dummy;
    left_dummy.y_min = y_max_limit;
    left_dummy.y_max = std::numeric_limits<float>::infinity();
    left_dummy.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    left_clusters.push_back(left_dummy);


    // שלב 5: מיון האשכולות לפי מיקום
    std::sort(left_clusters.begin(), left_clusters.end(), [](const ClusterInfo& a, const ClusterInfo& b) {
        return a.y_max > b.y_max; // משמאל לימין בצד השמאלי
        });

    std::sort(right_clusters.begin(), right_clusters.end(), [](const ClusterInfo& a, const ClusterInfo& b) {
        return a.y_min < b.y_min; // מימין לשמאל בצד הימני
        });

    // שלב 6: חיפוש מעבר פנוי
    bool found_passage = false;
    float window_size = 0.06; 

    float gap_middle = right_clusters[0].y_min - left_clusters.back().y_max;
    if (gap_middle >= window_size) {
        //me.setInstruction("Found passage between left and right clusters continue straight: " +std::to_string(gap_middle));
        return 0.0;
    }
    // נבדוק בין אשכולות בצד שמאל
    for (size_t i = 0; i + 1 < left_clusters.size()&&!found_passage; ++i)
    {
        float gap = left_clusters[i].y_min - left_clusters[i + 1].y_max;
        if (gap >= window_size)
        {
            float last = (i - 1 < 0) ? 0 : left_clusters[i].y_min - left_clusters[i - 1].y_max;
            
            //me.setInstruction( "Found passage in left: " + std::to_string(last) + " meters\n");
            found_passage = true;
            return last;
        }
    }

    // נבדוק בין אשכולות בצד ימין
        for (size_t i = 0; i + 1 < right_clusters.size()&&!found_passage; ++i)
        {
            float gap = right_clusters[i + 1].y_min - right_clusters[i].y_max;
            if (gap >= window_size)
            {
                float last=0;
                if (i - 1 >= 0)
                    last = right_clusters[i].y_max - right_clusters[i-1].y_max;
                //me.setInstruction( "Found passage in right: " +std::to_string( last) + " meters from you");
                found_passage = true;
                return last;
            }
        }

    if (!found_passage)
        me.setInstruction( "No passage found\n");

    return -1.1;
}
int instructionsObs(float x, float y, float distance, NevigationObject &me) {//cloud points is missed
	float space= findPassage(x,y, distance, me);
    std::string direction;
	if (space == -1.1) {
		me.setInstruction( "No passage found\n");
        //check if the node moves or throw down the edge and find another trail.
		return -1;
	}
    if (space == 0) {
	me.setInstruction( "continue straight");
    }
	else if (space > 0) {
        me.setInstruction( " left");
	}
	else {
		
       me.setInstruction(" right");
	}

}
pcl::PointCloud<pcl::PointXYZ>::Ptr transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Matrix4d T) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloud_transformed, T);
    return cloud_transformed;
}


