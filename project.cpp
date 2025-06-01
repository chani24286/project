//#define _CRT_SECURE_NO_WARNINGS
//#include <cstdio>
//#include <iostream>
//#include <memory>
//#include <stdexcept>
//#include <string>
//#include <array>
//#include <vector>
//#include <sstream>
//#include <nlohmann/json.hpp>
//#include <fstream>
//#include <algorithm>
//#include <cmath>
//#include "camToLidar.h"
//#include "obstacles.h"
//#include "global.h"
//float angle_global = 0.0;
//
//int stopping_length = 0;
////detected objects relevant for checking distances
//
//
//int listen() {
//    std::string output;
//    try {
//        std::cout << "runing speech recognition" << std::endl;
//	//	while (true) {
//			// Run the Python script to get the speech recognition result
//		std::string wellcome = "Hi, I am here to help you. If you want to go somewhere, just tell me and say go. for another service say other";
//        std::string command = "python \"C:\\Users\\User\\Documents\\projectC\\tts.py\"" + wellcome;
//        int result = system(command.c_str());
//
//	
//        output = exec("python \"C:\\Users\\User\\Documents\\projectC\\speech-recognition\\speechToText.py\"");
//        std::cout << "result: " << output << std::endl;
//    }
//    catch (const std::exception& e) {
//        std::cerr << "error: " << e.what() << std::endl;
//        return -1;
//    }
//    // Trim leading and trailing whitespace
//    output.erase(0, output.find_first_not_of(" \n\r\t"));
//    output.erase(output.find_last_not_of(" \n\r\t") + 1);
//
//    // Check if the text starts with "sens"
//    if (output.rfind("go", 0) == 0) { // rfind with 0 checks if it starts with "sens"
//        std::cout << "Input starts with 'other'" << std::endl;
//        split(output);
//    }
//}
//    int split(std::string output){
//        // Split the text into words
//        std::istringstream iss(output);
//        std::vector<std::string> words;
//        std::string word;
//        while (iss >> word) {
//            words.push_back(word);
//        }
//
//
//        // Check if the second word is "meet" and a name follows
//        if (words.size() >= 3 && words[1] == "meet") {
//            std::string personName = words[2]; // The third word is the person's name
//
//            // Run the Python script to save face images with the person's name
//
//            std::string str = "python \"C:\\Users\\User\\Documents\\projectC\\ML\\faceDetection\\faceDetection\\newFace.py\" \"" + personName + "\"";
//            char command[256];
//            strcpy(command, str.c_str());
//            std::string saveOutput = exec(command);
//            std::cout << "Started face capturing for: " << personName << std::endl;
//
//        }
//		if (words.size() >= 3 && words[1] == "go") {
//			std::string destination = words[2]; // The third word is the destination
//            //להפעיל קבלת מסלול עם היעד.
//
//            
//		}
//        return 0;
//    }
////while going
//    int going(std::string imagePath) {
//        std::string command = "python \"C:\\Users\\User\\Documents\\projectC\\ML.py\" \"" + imagePath + "\"";
//
//       
//           
//
//    }
//   
//
//    //road, downstairs, sidewalk, upstairs, pedestrian traffic light, red, green, crosswalk
//    int controller(object obj) {
//        initT();
//		initK();
//
//				std::string instructoins;
//                switch (obj.class_id) {
//                case(1):
//					instructoins = "downstairs are ahead";
//					break;
//				case(3):
//					instructoins = "upstairs are ahead";
//					break;
//				case(4):
//					//again ML not found the color of the traffic light
//					instructoins = "stop, traffic light";
//					break;
//				case(5):
//					instructoins = "stop, red light";
//					break;
//				case(6):
//					instructoins = "green light";
//					//to implement crosswalk function- calling to obstacles with the right parameters
//                    crosswalk();
//					break;
//				case(7):
//					instructoins = "crosswalk is ahead";
//                    crosswalk();
//					break;
//                }
//				std::string command = "python \"C:\\Users\\User\\Documents\\projectC\\tts.py\"" + instructoins;
//				int result = system(command.c_str());
//		
//			if(class_id==2){
//				const float* box = getBottomThird(bbox);
//				float min = minDistance(box);
//				float max = maxDistance(box);
//				findPassage(min, max, 2.0);
//			}
//		}
//		
//   }
//
//
//int main() 
//{
//    if (listen() == -1) {
//        std::cerr << "Error in speech recognition" << std::endl;
//        return -1;
//    }
//
//    return 0;
//}
#define _CRT_SECURE_NO_WARNINGS
#include <cstdio>
#include <iostream>
#include "NevigationObject.h"
#include "speach.h"
#include <thread>
#include <future>
#include "sensors.h"
#include "kalmanFilter.h"
#include "controller.h"
#include "obstacles.h"
int main() {
	std::string GPSpath = "C:\\Users\\User\\Documents\\projectC\\data\\GPS.txt";
	std::string LIDARpath = "C:\\Users\\User\\Documents\\projectC\\data\\lidar.pcd";
	std::string PEDOMETERpath = "C:\\Users\\User\\Documents\\projectC\\data\\pedometer.txt";
	std::string PICTUREpath = "C:\\Users\\User\\Documents\\projectC\\data\\0000000371 (1).png";
	NevigationObject me(0.0,0.0, 10.0);
	LidarSensor lidar;
	GPSsensor GPS;
	pedometer pedo;
	std::string result=speachAndSplit(me);
	if (result == "going") {
		std::thread lidarThread(&LidarSensor::setCurrent_scan,std::ref(lidar), LIDARpath);
		std::thread GPSThread(&GPSsensor::setLatAndLon, std::ref(GPS), GPSpath);
		std::thread pedometerThread(&pedometer::setSteps, std::ref(pedo), PEDOMETERpath);
		std::thread KFThread(runKalmanFilter, std::ref(GPS), std::ref(pedo), std::ref(lidar), std::ref(me));
		std::thread cameraThread(pictureDetection, std::ref(me), PICTUREpath, std::ref(lidar));
		std::thread FRCthread(faceRecognition, std::ref(me));
	}
}
