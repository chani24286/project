
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
#include <pcl/point_cloud.h>
#include "NevigationObject.h"
#include "sensors.h"

struct ClusterInfo {
    float y_min;   // הקצה הכי שמאלי באשכול (לפי Y)
    float y_max;   // הקצה הכי ימני באשכול (לפי Y)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // האשכול עצמו
};
pcl::PointCloud<pcl::PointXYZ>::Ptr filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float dist);
float findPassage(float y_min_limit, float y_max_limit, float dist, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, NevigationObject& me);
void instructionsObs(NevigationObject& me, LidarSensor& lidar);
void addFaeToKnownPeople(std::string name);
std::string faceRecognition(NevigationObject& me);
void blocked(NevigationObject& me, LidarSensor& lidar);