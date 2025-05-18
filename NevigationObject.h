#ifndef NAVIGATION_OBJECT_H
#define NAVIGATION_OBJECT_H

#include <array>
#include "controller.h"
#include "global.h"
#include "graph.h"
#include <vector>
#include <string>
class NevigationObject {
private:
    double nextTurnAngle;                  // Angle of the next turn (radians or degrees)
    double position;                       // Position along a 1D path (can be expanded to 2D/3D)
    std::vector<std::string> nowObstacles; // Boolean array: true if obstacle is present
    long long nextNode;
	double leftLidarDistance;
	double rightLidarDistance;
    std::string instructions;
    std::string additionalInstructions;
    std::vector<Edge> trail;
	Edge currentEdge; // Edge object to represent the current edge in the graph
public:
    // Constructor
    NevigationObject(double angle, double pos);
    //Funcptr* getFunctions();
    // Getter and Setter for nextTurnAngle
    double getNextTurnAngle() const;
    void setNextTurnAngle(double angle);
	void setNextNode(long long node);
    // Getter and Setter for position
    double getPosition() const;
    void setPosition(double pos);
    long long getNextNode() const;
    // Getter and Setter for nowObstacles
    std::vector <std::string> getNowObstacles() const;
    void setNowObstacles(std::string);
    void deleteNowObstacles(std::string obj);
	double getLeftLidarDistance() const;
	void setLeftLidarDistance(double distance);
	double getRightLidarDistance() const;
	void setRightLidarDistance(double distance);
	// Getter for trail
    std::vector<Edge> getTrail()const;
	// Setter for trail
    void setTrail(const std::vector<Edge>& newTrail);
    void setCurrentEdge(const Edge& edge);
    Edge getCurrentEdge()const;
	Edge goToNextEdge();
	void clculateAngle();
    void setInstruction(std::string instruction);
    std::string getInstruction();
    void setAdditionalInstructions(std::string instruction);
    std::string getAdditionalInstructions();
using Funcptr = void(*)(object, NevigationObject); // Function pointer type for handling yolo detections
Funcptr functions[8] = { road, downstairs, sidewalk, upstairs, trafficlight, red, green, crosswalk }; // Array of function pointers for yolo  handling


#endif // NAVIGATION_OBJECT_H
