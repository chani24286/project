
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
#include "obstacles.h"
#include "sensors.h"
#include <thread>
#include <chrono>
#include "graph.h"
#include <Eigen/Dense>


pcl::PointCloud<pcl::PointXYZ>::Ptr filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float dist) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(2.0, 2.0);
    pass.filter(*temp_cloud);

    pass.setInputCloud(temp_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, dist);
    pass.filter(*cloud); 

    return cloud;
}

float findPassage(float y_min_limit, float y_max_limit, float dist, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, NevigationObject &me)
{
 
    cloud = filter(cloud, dist);
    std::vector<pcl::PointIndices> cluster_indices;
    if (!cloud->empty()) {
    
    // שלב 2: יצירת KDTree לאשכולות
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // שלב 3: חיפוש אשכולות
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3); // מרחק מקסימלי בין נקודות כדי להיחשב באותו אשכול
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    }
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
        me.setInstruction("Found passage between left and right clusters continue straight: " +std::to_string(gap_middle));
        return 0.0;
    }
    // נבדוק בין אשכולות בצד שמאל
    for (size_t i = 0; i + 1 < left_clusters.size()&&!found_passage; ++i)
    {
        float gap = left_clusters[i].y_min - left_clusters[i + 1].y_max;
        if (gap >= window_size)
        {
            float last = (i - 1 < 0) ? 0 : left_clusters[i].y_min;
            
			std::cout << "Found passage in left: " << last << " meters from you\n";
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
                    last = right_clusters[i].y_max;
				std::cout << "found passage in right: " << last << " meters from you\n";
                found_passage = true;
                return last;
            }
        }

    if (!found_passage)
        me.setInstruction( "No passage found\n");

 return -1.1;
}
void instructionsObs( NevigationObject &me,LidarSensor &lidar ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar.getCurrent_scan();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    float left = me.getLeftLidarDistance();
    float right = me.getRightLidarDistance();
    float distance = me.getLengthLidar();
    if (me.getObstacles()) {
        Eigen::Matrix4f T;
        T<< 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
            1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
            9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
            0.0, 0.0, 0.0, 1.0;
        try {
            pcl::transformPointCloud(*cloud, *transformed_cloud, T);
            std::cout << "Transform completed, got " << transformed_cloud->size() << " points\n";
        }
        catch (const std::exception& e) {
            std::cerr << "Transform failed: " << e.what() << std::endl;
        }
        float space = findPassage(left, right, distance, cloud, me);
        std::string direction;
        if (space == -1.1) {
            blocked(me, lidar);
        }
        if (space == 0) {
            me.setInstruction("continue straight");
        }
        else if (space > 0) {
            me.setInstruction(" turn left");
        }
        else {

            me.setInstruction(" turn right");
        }
    }
    else {
        if (me.getLeftLidarDistance() < 0.2)
            me.setInstruction("you are too close to left");
        if (me.getRightLidarDistance() < 0.2)
            me.setInstruction("you are too close to right");
    }
       
}
void blocked(NevigationObject& me, LidarSensor& lidar) {
    bool waitedMuch = false;
    if (lidar.getIsNoisy()) {
        me.setInstruction("Stop, the system has detected an obstacle that seems to move away soon.");
        if (lidar.getStartTime() == std::chrono::system_clock::time_point::min())
            lidar.setStartTime(std::chrono::system_clock::now());
        else {
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = now - lidar.getStartTime();
            if (diff < std::chrono::seconds(4))
                waitedMuch = true;
        }
    }
    if(!lidar.getIsNoisy() || waitedMuch){
        Node last = me.getLastNode();
        Edge lastEdge = me.getCurrentEdge();   
        double constDist = lastEdge.distance - me.getPosition();
        remove_edge(me.getLastNode().id, me.getNextNode().id);
        long long target = me.getTargetID();
        std::unordered_map<long long, long long> previous;
        auto distances = dijkstra(last.id, previous);
        std::vector<Edge> allpath = reconstruct_path(last.id, target, previous);
        me.setTrail(allpath);
        me.setLastNode(nodes[last.id]);
        me.addEdgeToFront(Edge{ last.id, constDist, lastEdge.name });
        me.setInstruction("the way is blocked, calculate a new trail");
        
    }
}
//pcl::PointCloud<pcl::PointXYZ>::Ptr transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Matrix4d T) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::transformPointCloud(*cloud, *cloud_transformed, T);
//    return cloud_transformed;
//}
std::string faceRecognition(NevigationObject& me) {
    std::string command = "python \"C:\\Users\\User\\Documents\\projectC\\ML\\faceDetection\\faceDetection\\detection.py\"";
    std::string result = exec(command.c_str());
    std::string known = me.getKnownPeople();
    if (known != result)
    {
        me.setKnownPeople(result);
        return result;
    }

}
void addFaeToKnownPeople(std::string name) {
    std::string command = "python \"C:\\Users\\User\\Documents\\projectC\\ML\\faceDetection\\faceDetection\\detection.py\"" + name;
    std::string result = exec(command.c_str());
}

