#pragma once
#include <iostream>
#include <string>
#include "NevigationObject.h"
#include <vector>
#include "sensors.h"
struct object {
    float angle;
    float bbox[4];
    int class_id;
    // הגדרת אופרטור השוואה
    bool operator==(object& other) const {
        return angle == other.angle &&
            class_id == other.class_id &&
            bbox[0] == other.bbox[0] &&
            bbox[1] == other.bbox[1] &&
            bbox[2] == other.bbox[2] &&
            bbox[3] == other.bbox[3];
    }
};

bool relevant(object& obj, NevigationObject& me);
void findLimits(float bbox[], NevigationObject& me);
void clearLock(std::vector <object>& objects, NevigationObject& me);
void pictureDetection(NevigationObject& me, std::string path, LidarSensor& lidar);
std::vector <object> readYolo(NevigationObject& me);
void sidewalk(object& obj, NevigationObject& me, LidarSensor& lidar);
void crosswalk(object& obj, NevigationObject& me, LidarSensor& lidar);
void green(object& obj, NevigationObject& me, LidarSensor& lidar);
void upstairs(object& obj, NevigationObject& me, LidarSensor& lidar);
void downstairs(object& obj, NevigationObject& me, LidarSensor& lidar);
void red(object& obj, NevigationObject& me, LidarSensor& lidar);
void road(object& obj, NevigationObject& me, LidarSensor& lidar);
void trafficlight(object& obj, NevigationObject& me, LidarSensor& lidar);

using Funcptr = void(*)(object&, NevigationObject&, LidarSensor& ); // Function pointer type for handling yolo detections
extern Funcptr functions[8];

