#include "NevigationObject.h"
#include <iostream>
#include "graph.h"
#include "global.h"
#include <string>

// Constructor
NevigationObject::NevigationObject(double angle, double pos)
    : nextTurnAngle(angle), position(pos) {
}

// Getters
double NevigationObject::getNextTurnAngle() const {
    return nextTurnAngle;
}

double NevigationObject::getPosition() const {
    return position;
}

std::vector <std::string> NevigationObject::getNowObstacles() const {
    return nowObstacles;
}
long long NevigationObject::getNextNode() const {
	return nextNode;
}
double NevigationObject::getLeftLidarDistance() const {
	return leftLidarDistance;
}
double NevigationObject::getRightLidarDistance() const {
	return rightLidarDistance;
}
//return array of function pointers
//NevigationObject::Funcptr* NevigationObject::getFunctions() {
//    return functions;
//}
std::vector<Edge> NevigationObject::getTrail() const { return trail; }

Edge NevigationObject::getCurrentEdge() const { return currentEdge; }
Edge NevigationObject::goToNextEdge() {
	// Assuming the next edge is the first one in the trail
	if (!trail.empty()) {
		Edge edge = trail.front(); // Return the first edge in the trail
		trail.erase(trail.begin()); // Remove the first edge from the trail
		setPosition(edge.distance);
		setNextNode(edge.to); // Update the next node
		return edge;
	}
	return Edge(); // Return a default Edge if trail is empty
}


// Setters
void NevigationObject::setNextTurnAngle(double angle) {
    nextTurnAngle = angle;
}

void NevigationObject::setPosition(double pos) {
    position = pos;
}

void NevigationObject::setNowObstacles(std::string ob) {
	nowObstacles.push_back(ob);
}
void NevigationObject::deleteNowObstacles(std::string obj) {
	auto it = std::remove(nowObstacles.begin(), nowObstacles.end(), obj);
	if (it != nowObstacles.end()) {
		nowObstacles.erase(it, nowObstacles.end());
	}
}
void NevigationObject::setNextNode(long long node) {
	nextNode = node;
}
void NevigationObject::setLeftLidarDistance(double distance) {
	leftLidarDistance = distance;
}
void NevigationObject::setRightLidarDistance(double distance) {
	rightLidarDistance = distance;
}
void NevigationObject::setTrail(const std::vector<Edge>& newTrail) { trail = newTrail; }

void NevigationObject::setCurrentEdge(const Edge& edge) { currentEdge = edge; }
void NevigationObject::clculateAngle() {
	// Assuming you want to calculate the angle based on the current edge
	if (!currentEdge.name.empty()) {
		nextTurnAngle = currentEdge.angle; // Set the next turn angle based on the current edge
	}
}