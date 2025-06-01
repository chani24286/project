#ifndef NAVIGATION_OBJECT_H
#define NAVIGATION_OBJECT_H

#include <array>
#include "global.h"
#include <vector>
#include <string>
#include <mutex>
class NevigationObject {
private:
    double stepLength;
    double nextTurnAngle;                  // Angle of the next turn (degrees)
    double position;                       // Position along a 1D path 
    std::vector<int> nowObstacles; // int array: classes ids are in if obstacle is present
    double leftLidarDistance;
    double rightLidarDistance;
    double lengthLidar;
    bool obstacles;
    std::string instructions;
    std::string additionalInstructions;
    std::vector<Edge> trail;
    Edge currentEdge; // Edge object to represent the current edge in the graph
    Node nextNode;
    Node lastNode;
    std::string knowPeople;
    std::mutex mtx;
public:
    // Constructor
    NevigationObject(double angle, double pos, double step);
    //Funcptr* getFunctions();
    // Getter and Setter for nextTurnAngle
    double getStepLength();
    void setStepLength(double step);

    double getNextTurnAngle() const;
    void setNextTurnAngle(double angle);
    // Getter and Setter for position
    double getPosition() const;
    void setPosition(double pos);
    // Getter and Setter for nowObstacles
    std::vector <int> getNowObstacles() const;
    void setNowObstacles(int obj);
    void deleteNowObstacles(int obj);
    double getLeftLidarDistance() const;
    void setLeftLidarDistance(double distance);
    double getRightLidarDistance() const;
    void setRightLidarDistance(double distance);
    double getLengthLidar();
    void setLengthLidar(double length);
    void setObstacles(bool notInstructed);
    bool getObstacles();
    std::vector<Edge>& getTrail();
    long long getTargetID();
    void initAngle();
    void setTrail(const std::vector<Edge>& newTrail);
    void setCurrentEdge(const Edge& edge);
    Edge getCurrentEdge()const;
    void goToNextEdge();
    Node getNextNode();
    void setNextNode(Node newNode);
    Node getLastNode();
    void setLastNode(Node newNode);
    void setInstruction(std::string instruction);
    std::string getInstruction();
    void setAdditionalInstructions(std::string instruction);
    std::string getAdditionalInstructions();
    void addEdgeToFront(Edge newOne);
    std::string getKnownPeople();
    void setKnownPeople(std::string people);
};

#endif // NAVIGATION_OBJECT_H
