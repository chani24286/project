#include "sensors.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include "global.h"
#include "ICP.h"
#include <Eigen/Dense>
#include <chrono>
#include <thread>


// Sensor methods
Sensor::Sensor() : total_distance(0.0), total_time(0.0) {}

void Sensor::setTotalDistance(double distance) {
    total_distance = distance;
}

void Sensor::setTotalTime(double time) {
    total_time = time;
}

double Sensor::getTotalDistance() const {
    return total_distance;
}

double Sensor::getTotalTime() const {
    return total_time;
}

//Sensor::~Sensor() {}


// LidarSensor methods
LidarSensor::LidarSensor() : latest_scan(new pcl::PointCloud<pcl::PointXYZ>) {}

void LidarSensor::setCurrent_scan(std::string path) {
	latest_scan = current_scan;
	current_scan = lidarReading(path);
	//total_distance = icpAlgorithm(latest_scan, current_scan);
	ICP_out out= icpAlgorithm(latest_scan, current_scan);
	setMotion(out);
	std::this_thread::sleep_for(std::chrono::seconds(3));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarSensor::getLatest_scan()  {
	return latest_scan;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr LidarSensor::getCurrent_scan() {
	return current_scan;
}
void LidarSensor:: setMotion(ICP_out newMotion) {
	motion = newMotion;
	Eigen::Vector3d translation = newMotion.T.block<3, 1>(0, 3);
	double dx = translation(0);  // המרחק בציר X
	double dy = translation(1);  // המרחק בציר Y
	total_distance = std::sqrt(dx * dx + dy * dy);
	printing("icp - motion recognized: " + std::to_string(total_distance));
}
ICP_out LidarSensor :: getMotion() {
	return motion;
}
bool LidarSensor::getIsNoisy() {
	return isNoisey;
}
std::chrono::system_clock::time_point LidarSensor::getStartTime() const {
	return startTime;
}
void LidarSensor::setIsNoisey() {
	isNoisey = movements(motion);
}
void LidarSensor::setStartTime(std::chrono::system_clock::time_point newTime) {
	startTime = newTime;
}
// GPSsensor methods
GPSsensor::GPSsensor() : lastLat(0.0), lastLon(0.0) {}
//void GPSsensor::setLastLatLon(double lat, double lon) {
//	lastLat = lat;
//	lastLon = lon;
//}
double GPSsensor::getLat() {
	return lastLat;
}
double GPSsensor::getLon() {
	return lastLon;
}
void GPSsensor::setLatAndLon(std::string path) {
	//reading from file GPS
	std::ifstream file(path);
	if (!file.is_open()) {
		throw std::runtime_error("Failed to open file: " + path);
	}

	file >> lat >> lon;

	if (file.fail()) {
		throw std::runtime_error("Failed to read two doubles from file.");
	}
	lastLat = lat;
	lastLon = lon;
	double dist = haversine(lastLat, lastLon, lat, lon);
	setTotalDistance(dist); // חישוב המרחק הכולל
	printing("GPS recognized motion: " + std::to_string(dist));
	std::this_thread::sleep_for(std::chrono::seconds(7));
}

// Pedometer methods
pedometer::pedometer() : MFS(0.1) {}
void pedometer::setMFS(double length, int numSteps) {
	MFS = length / numSteps; // חישוב אורך צעד
}
double pedometer::getMFS() {
	return MFS;
}

int pedometer::getSteps() {
	return steps;
}
double pedometer::getSpeedness() {
	return MFS * steps; // חישוב מהירות על פי אורך הצעד
}
void pedometer::setSteps(std::string path) {
	std::ifstream file(path);
	if (!file.is_open()) {
		throw std::runtime_error("Failed to open file: " + path);
	}

	file >> steps;

	if (file.fail()) {
		throw std::runtime_error("Failed to read two doubles from file.");
	}
	printing("pedometer recognized walking speednes: " + std::to_string(steps));
	setTotalDistance(getTotalTime() * getSpeedness());
	std::this_thread::sleep_for(std::chrono::seconds(7));
}
double pedometer::getStepsLength(double distance) {
	return distance / getSpeedness(); // חישוב אורך הצעד על פי המרחק
}


//GPSData getSimulatedGPS() {
//    // עושים תזוזה קטנה בכל קריאה
//    currentLatitude += 0.0001;
//    currentLongitude += 0.0001;
//
//    return { currentLatitude, currentLongitude };
//}
