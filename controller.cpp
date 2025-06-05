#define _CRT_SECURE_NO_WARNINGS

#include <immintrin.h> 
#include "controller.h"
#include <iostream>
#include <string>
#include <array>
#include "camToLidar.h"
#include "obstacles.h"
#include "NevigationObject.h"
#include <vector>
#include <sstream>
#include <nlohmann/json.hpp>
#include <algorithm>
#include "graph.h"
#include "global.h"
#include "sensors.h"

Funcptr functions[8] = { road, downstairs, sidewalk, upstairs, trafficlight, red, green, crosswalk };
void runNewLidar(NevigationObject& me, LidarSensor& lidar) {
	std::string LIDARpath = "C:\\Users\\User\\Documents\\projectC\\data\\lidar1.pcd";
	std::thread cameraThread(pictureDetection, std::ref(me), std::ref(lidar));
	std::thread lidarThread(&LidarSensor::setCurrent_scan, std::ref(lidar), LIDARpath);
	lidarThread.join();
	cameraThread.join();
	LIDARpath = "C:\\Users\\User\\Documents\\projectC\\data\\lidar1.pcd";
}
void camAndLidar(NevigationObject& me, LidarSensor& lidar) {

	try {
		std::string lastContent = "";
		while(true){
		std::ifstream file("C:\\Users\\User\\Documents\\projectC\\detections_video.json");
		if (!file.is_open()) {
			return ; 
		}
		std::stringstream buffer;
		buffer << file.rdbuf();
		std::string currentContent = buffer.str();
		if (currentContent != lastContent)
			runNewLidar(me, lidar);
		}
		
	}
	catch (const std::exception& e) {
		std::cerr << "Error reading file: " << e.what() << std::endl;
		return;
	}
}
std::vector <object> readYolo(NevigationObject &me) {
 // output from yolo-JSON
	std::this_thread::sleep_for(std::chrono::milliseconds(30));
    nlohmann::json detections;
    try {
        std::ifstream("C:\\Users\\User\\Documents\\projectC\\detections.json") >> detections;
		std::cerr << "Content: " << detections.dump(4) << std::endl;
		if (!detections.is_array()) {
			std::cerr << "Error: JSON detections is not an array. Actual type: " << detections.type_name() << std::endl;
		}
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
		return std::vector < object>();
    }
    if (detections.empty()) {
		printing("No data found in output.json");
		 return std::vector < object>();
    }
	std::vector<object> objects;	
    //scan each detection, clculate closest angle from the detected object
    for (const auto& det : detections) {
        int class_id = det["class_id"];
        float confidence = det["confidence"];
        float bbox[4] = { det["bbox"][0], det["bbox"][1], det["bbox"][2], det["bbox"][3] };
		object obj{ 0.0, { bbox[0], bbox[1], bbox[2], bbox[3] }, class_id };
		printing("detected in picture class number: " + std::to_string(class_id));
		//if (relevant(obj, me)){
			objects.push_back(obj);
			for (size_t i = 0; i < objects.size(); ++i) {
				const auto& obj = objects[i];
				printing("Object " + std::to_string(i + 1) + ":");
				printing("  Class ID: " + std::to_string(obj.class_id));
				printing("  Bounding Box: [" + std::to_string(obj.bbox[0]) + ", " +
					std::to_string(obj.bbox[1]) + ", " +
					std::to_string(obj.bbox[2]) + ", " +
					std::to_string(obj.bbox[3]) + "]");
				// You can add more fields to print if your 'object' struct has them
			//}
		}
    }
	return objects;
}
void pictureDetection(NevigationObject &me, LidarSensor& lidar) {
	std::vector <object> objects = readYolo(me);
	clearLock(objects, me);
	for ( auto& obj : objects) {
		functions[obj.class_id](obj, me, lidar);
	}	
}
void yoloDetection(std::string path, NevigationObject &me, LidarSensor &lidar) {
	system(path.c_str());
	camAndLidar(me, lidar);
}

void clearLock(std::vector<object>& objects, NevigationObject& me) {
	std::vector<int> currentObstacles = me.getNowObstacles();
	for (const auto& existing_obj : currentObstacles) {
		auto it = std::find_if(objects.begin(), objects.end(),
			[existing_obj](const object& obj) {
				return obj.class_id == existing_obj;
			});
		me.deleteNowObstacles(existing_obj);
	}
}

void findLimits(float bbox[], NevigationObject &me) {
	me.setLeftLidarDistance (leftDistance(bbox));
	me.setRightLidarDistance( rightDistance(bbox));
	std::cout << "Left distance: " << me.getLeftLidarDistance() << " Right distance: " << me.getRightLidarDistance() << std::endl;
}


bool relevant(object &obj, NevigationObject &me) {
	obj.angle = angle(obj.bbox) - me.getNextTurnAngle();
	std::cout << "angle: " << obj.angle << " for class: " << obj.class_id << std::endl;
	return (std::fabs(obj.angle) > 30.0);
}
//road, downstairs, sidewalk, upstairs, pedestrian traffic light, red, green, crosswalk
void downstairs(object& obj, NevigationObject& me, LidarSensor& lidar) {
	std::vector<int> objects = me.getNowObstacles();
	auto it = std::find(objects.begin(), objects.end(), obj.class_id);

	if (it == objects.end()) { // רק אם עוד לא קיים
		me.setAdditionalInstructions("downstairs are ahead");
		me.setNowObstacles(obj.class_id);
		me.setLengthLidar(0.0);
		me.setObstacles(false);
	}

	findLimits(obj.bbox, me);
}


	
void upstairs(object& obj, NevigationObject& me, LidarSensor& lidar) {
	std::vector <int> objects = me.getNowObstacles();
	auto it = std::find(objects.begin(), objects.end(), obj.class_id);

	if (it == objects.end()) { // רק אם עוד לא קיים
		me.setAdditionalInstructions("upstirs are ahead");
		me.setNowObstacles(obj.class_id);
		me.setLengthLidar(0.0);
		me.setObstacles(false);
	}

	findLimits(obj.bbox, me);
}
void red(object& obj, NevigationObject& me, LidarSensor& lidar) {
	//check wether the trail says there is a crossing
	if (me.getCurrentEdge().name != "traffic signals")
		return;
	std::vector <int> objects = me.getNowObstacles();
	auto it = std::find(objects.begin(), objects.end(), obj.class_id);
	if (it == objects.end()) {
		me.setInstruction("stop, red traffic light");
		me.setNowObstacles(obj.class_id);
		me.setLengthLidar(0.0);
		me.setObstacles(false);
	}
	findLimits(obj.bbox, me);
}
void green(object &obj, NevigationObject &me, LidarSensor& lidar) {
	if (me.getCurrentEdge().name!= "traffic signals")
		return;
	std::vector <int> objects = me.getNowObstacles();
	auto it = std::find(objects.begin(), objects.end(), obj.class_id);
	if (it != objects.end()) {
		me.setAdditionalInstructions("green traffic light");
		me.setNowObstacles(obj.class_id);
	}
}
void crosswalk(object& obj, NevigationObject& me, LidarSensor& lidar) {
	if (me.getCurrentEdge().name == "crossing")
		return;
	std::vector <int> objects = me.getNowObstacles();
	auto it = std::find(objects.begin(), objects.end(), obj.class_id);
	if (it == objects.end()) {
		me.setAdditionalInstructions("croswalk");
		me.setObstacles(true);
		me.setNowObstacles(obj.class_id);
	}
	findLimits(obj.bbox, me);
	me.setLengthLidar(maxDistance(obj.bbox));
	instructionsObs(me, lidar);
}
void sidewalk(object &obj, NevigationObject &me, LidarSensor& lidar) {
	findLimits(obj.bbox, me);
	me.setLengthLidar(0.2);
	me.setObstacles(true);
	instructionsObs(me, lidar);
}
void road(object& obj, NevigationObject& me, LidarSensor& lidar) {
	me.setAdditionalInstructions("you are on the road!! go to left ro to the right to reach sidewalk ");
	me.setObstacles(false);
}
void trafficlight(object& obj, NevigationObject& me, LidarSensor& lidar) {
	std::vector <int> objects = me.getNowObstacles();
	auto it = std::find(objects.begin(), objects.end(), obj.class_id);
	if (it != objects.end())
		me.setAdditionalInstructions("traffic light is detected but the light color is not recognize yet");
}
