#define _CRT_SECURE_NO_WARNINGS

#include "NevigationObject.h"
#include <iostream>
#include "global.h"
#include <string>
#include <mutex>
#include "speach.h"
#include <limits>
// Constructor
NevigationObject::NevigationObject(double angle, double pos, double step)
    : nextTurnAngle(angle), position(pos), stepLength(step) {
}
double NevigationObject::getStepLength() {
	return stepLength;
}
// Getters
double NevigationObject::getNextTurnAngle() const {
    return nextTurnAngle;
}

double NevigationObject::getPosition() const {
    return position;
}

std::vector <int> NevigationObject::getNowObstacles() const {
    return nowObstacles;
}
Node NevigationObject::getNextNode() {
	return nextNode;
}
Node NevigationObject::getLastNode() {
	return lastNode;
}
double NevigationObject::getLeftLidarDistance() const {
	return leftLidarDistance;
}
double NevigationObject::getRightLidarDistance() const {
	return rightLidarDistance;
}
double NevigationObject::getLengthLidar() {
	return lengthLidar;
}
bool NevigationObject::getObstacles() {
	return obstacles;
}
std::string NevigationObject::getKnownPeople() {
	return knowPeople;
}
//return array of function pointers
//NevigationObject::Funcptr* NevigationObject::getFunctions() {
//    return functions;
//}
std::vector<Edge>& NevigationObject::getTrail() { return trail; }

Edge NevigationObject::getCurrentEdge() const { return currentEdge; }
void NevigationObject::goToNextEdge() {
	// Assuming the next edge is the first one in the trail
	if (!trail.empty()) {
		Edge edge = trail.front(); // Return the first edge in the trail
		trail.erase(trail.begin()); // Remove the first edge from the trail
		setPosition(edge.distance);
		setLastNode(nextNode);
		setNextNode(nodes[edge.to]); // Update the next node
		setCurrentEdge(edge);
		initAngle();
		return;
	}
	sayIt("got it! you reached the target!!");
	keepRunning = false;
	return ; 
}
void NevigationObject::addEdgeToFront(Edge newOne) {
	getTrail().insert(getTrail().begin(), newOne);
}
long long NevigationObject::getTargetID() {
	return trail.back().to;
}
std::string NevigationObject::getInstruction() {
	return instructions;
}
std::string NevigationObject::getAdditionalInstructions() {
	return additionalInstructions;
}

// Setters
void NevigationObject::setStepLength(double step) {
	stepLength = step;
}
void NevigationObject::setNextTurnAngle(double angle) {
	std::lock_guard<std::mutex> lock(mtx);
    nextTurnAngle = angle;
}
void NevigationObject::initAngle() {
	setNextTurnAngle(0.0);
}
void NevigationObject::setPosition(double pos) {
    position = pos;
	printing("Current position: " + std::to_string(position) + "m");
}

void NevigationObject::setNowObstacles(int ob) {
	nowObstacles.push_back(ob);
	printing("Obstacle added: " + std::to_string(ob));
}
void NevigationObject::deleteNowObstacles(int obj) {
	auto it = std::remove(nowObstacles.begin(), nowObstacles.end(), obj);
	if (it != nowObstacles.end()) {
		nowObstacles.erase(it, nowObstacles.end());
	}
}
void NevigationObject::setNextNode(Node newNode) {
	nextNode = newNode;
	printing("Next node set to: " + std::to_string(newNode.id) + " at position (" + std::to_string(newNode.lat) + ", " + std::to_string(newNode.lon) + ")");
}
void NevigationObject::setLastNode(Node newNode) {
	lastNode = newNode;
}
void NevigationObject::setLeftLidarDistance(double distance) {
	leftLidarDistance = distance;
	printing("Left Lidar distance set to: " + std::to_string(leftLidarDistance) + "m");
}
void NevigationObject::setRightLidarDistance(double distance) {
	rightLidarDistance = distance;
	printing("Right Lidar distance set to: " + std::to_string(rightLidarDistance) + "m");
}
void NevigationObject::setLengthLidar(double length) {
	lengthLidar = length;
	printf("Lidar length set to: %.2f m\n", lengthLidar);
}
void NevigationObject::setObstacles(bool notInstructed) {
	obstacles = notInstructed;
}
void NevigationObject::setTrail(const std::vector<Edge>& newTrail) { 
	trail = newTrail; 
	printing("got new trail");
	goToNextEdge();
}

void NevigationObject::setCurrentEdge(const Edge& edge) { 
	currentEdge = edge; 
	setNextNode(nodes[currentEdge.to]);
}

void NevigationObject::setAdditionalInstructions(std::string instruction) {
	std::lock_guard<std::mutex> lock(mtx);
	additionalInstructions = instruction;
	sayIt(additionalInstructions);
	printing( additionalInstructions);
}
void NevigationObject::setInstruction(std::string instruction) {
	std::lock_guard<std::mutex> lock(mtx);
	this->instructions = instruction;
	sayIt(this->instructions);
	printing(this->instructions);
}
void NevigationObject::setKnownPeople(std::string people) {
	knowPeople = people;
	sayIt( people+"is in the area");
	printing("Known people updated: " + knowPeople);
}

