#include "controller.h"
#include <iostream>
#include <string>
#include <array>
#include "camToLidar.h"
#include "obstacles.h"
#include "NevigationObject.h"
#include <array>
#include <vector>
#include <sstream>
#include <nlohmann/json.hpp>
//include "graph.h"
//the next two items needs to be included in the sense object
std::string instructoins;
std::vector <object> readYolo(NevigationObject me) {

    // ניתוח הפלט כ-JSON
    nlohmann::json detections;
    try {
        std::ifstream("output.json") >> detections;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    if (detections.empty()) {
        std::cerr << "No data found in output.json" << std::endl;
        return error;
    }
	std::vector<object> objects;	
    //scan each detection, clculate closest angle from the detected object
    for (const auto& det : detections[0]) {
        int class_id = det["class_id"];
        float confidence = det["confidence"];
        float bbox[4] = { det["bbox"][0], det["bbox"][1], det["bbox"][2], det["bbox"][3] };
		if (relevant(obj, me))
			//calling to the match function, include building 'object' with the right parameters
			objects.push( object(0.0, { bbox[0], bbox[1], bbox[2], bbox[3] }, class_id));
    }
	return objects;
}
void callingFunctions(NevigationObject me) {
	std::vector <objects> obj = readYolo();
	clearLock(obj, me);
	for (const auto& obj : objects) {
		functions[obj.class_id](obj, me);
	}
	
}
void clearLock(vector <object> objects, NevigationObject me) {
	for (auto obj : me.getNowObstacles()) {
		if (objects.find(obj) == objects.end()) {
			me.deleteNowObstacles(obj);
		}
	}
	
}
std::pair<float, float> findLimits(float bbox[]) {
	std::pair<float, float> limits;
	limits.first = leftDistance(bbox);
	limits.second = rightDistance(bbox);
	instructions = (limiits.first <= 20.0) ? "too close to left" : (limits.second<=20.0)?"too close to right";
	return limits;
}


bool relevant(object obj, NevigationObject me) {
	obj.angle = angle(obj.bbox) - me.getNextTurnAngle();
	return (std::fabs(obj.angle) > 30.0);
}
//road, downstairs, sidewalk, upstairs, pedestrian traffic light, red, green, crosswalk
void downstairs(object obj) {
		me.setNowObstacles(obj);
		instructoins = "downstairs are ahead";
		//calling to tts function
		limits = findLimits(obj.bbox);
			//calling to too close limits function
		}
		
	}
	
}
void upstairs(object obj) {	
	me.setNowObstacles(obj);
	instructoins = "upnstairs are ahead";
	limits = findLimits(obj.bbox);
	}
}
void red(object obj) {
	
	//check wether the trail says there is a crossing
	if (!nodes[me.getNextNode()].crosswalk) {
		continue;
	me.setNowObstacles(obj);
	instructoins = "stop, red trafficlight";

	}
}
	void green(object obj, NevigationObject me) {			
			//chek if the trail says there is a traffic light
			me.setNowObstacles(obj);
			instructoins = "green traffic light";
			instructionsObs(rightLimits(obj.bbox), leftLimits(obj.bbox), maxDistance(bbox));				

	}
	void crosswalk(object obj, NevigationObject me) {
			//check in the trail if there is a crosswlk
			me.setNowObstacles(obj);
			instructoins = "crosswalk";
		
		instructionsObs(rightLimits(obj.bbox), leftLimits(obj.bbox), maxDistance(bbox));

	}
	void sidewalk(object obj, NevigationObject me) {
		instructionsObs(rightLimits(obj.bbox), leftLimits(obj.bbox), 20.0);
	}

void controll(NevigationObject me) {
	readYolo();
}
