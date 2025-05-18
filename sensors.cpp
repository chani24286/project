#include "sensors.h"
#include <cmath>
#include <iostream>
#include <string>
#include "global.h"
#include "ICP.h"
#include <Eigen/Dense>
#define M_PI 3.14159265358979323846


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
	Eigen::MatrixXd T= icpAlgorithm(latest_scan, current_scan);
	setMotion(T);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarSensor::getLatest_scan()  {
    return latest_scan;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr LidarSensor::getCurrent_scan() {
	return current_scan;
}
void LidarSensor:: setMotion(ICP_out newMotion) {
	motion = newMotion;
	Vector3d translation = newMotion.T.block<3, 1>(0, 3);
	double dx = translation(0);  // ����� ���� X
	double dy = translation(1);  // ����� ���� Y
	//double dist=math.pow?
	//total_distance = dist;
}
Eigen::MatrixXd LidarSensor :: getMotion() {
	return motion;
}

// GPSsensor methods
GPSsensor::GPSsensor() : lastLat(0.0), lastLon(0.0) {}
void GPSsensor::setLastLatLon(double lat, double lon) {
	lastLat = lat;
	lastLon = lon;
}
double GPSsensor::getLat() {
	return lastLat;
}
double GPSsensor::getLon() {
	return lastLon;
}
void GPSsensor::setLatAndLon(std::string path) {
	// ����� ������ ������
	//reading from file GPS
	lastLat = lat;
	lastLon = lon;
	setTotalDistance(haversine(lastLat, lastLon, lat, lon)); // ����� ����� �����
}

// Pedometer methods
pedometer::pedometer() : MFS(0.1) {}
void pedometer::setMFS(double length, int numSteps) {
	MFS = length / numSteps; // ����� ���� ���
}
double pedometer::getMFS() {
	return MFS;
}

int pedometer::getSteps() {
	return steps;
}
double pedometer::getSpeedness() {
	return MFS * steps; // ����� ������ �� �� ���� ����
}
void pedometer::setSteps(std::string path) {
	// ����� ���� ������
	//reading from file steps
	setTotalDistance(getTotalTime() * getSpeedness());
}
double pedometer::getStepsLength(double distance) {
	return distance / getSpeedness(); // ����� ���� ���� �� �� �����
}


//GPSData getSimulatedGPS() {
//    // ����� ����� ���� ��� �����
//    currentLatitude += 0.0001;
//    currentLongitude += 0.0001;
//
//    return { currentLatitude, currentLongitude };
//}
