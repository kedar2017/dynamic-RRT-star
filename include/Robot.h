#pragma once
#include <utility>
#include <mutex>
#include <iostream>
#include <vector>
#include <Space.h>

using namespace std;
class World;

class Robot{
public:
    double x, y;            // in m
    const double vel;       // in m/s
    Node* nextDestination; // goal buffer
    Node* goalNode;             // goal node
    Node* currNode;       // current node at which robot is present
    Node* localGoal;       // replanner goal

    double E;

    World* world;   // handle to the world
    
    std::vector<Node*> pathtoGoal;     // current plan
    std::vector<Node*> globalTree; // repair plan

    std::mutex poseMtx;
    std::mutex destMtx;

    // list of ctors
    Robot();
    Robot(double x, double y, double vel);
    Robot(double x, double y, double vel, double x_goal, double y_goal);
    
    // Getters
    bool getGoal(double& gx, double& gy);
    void getRobotPose(double &rx, double &ry);
    Node* getDestination();
    double getVel();
    std::vector<Node*> getCurrentTree();
    std::vector<Node*> getLocalTree();
    std::vector<Node*> getPlan();
    Node* getRobotNode();
    Node* getReplanGoal();

    // Setters
    void setWorld(World* world);                            // main links robot to world
    void setRobotNode(Node* node);    // world updates robot pose
    void setDestination(Node* dest);                        // planner should update destination

    bool robotAtGoal();
    bool robotAtDestination();
};

Robot::Robot()
    : x(0), y(0), vel(0), nextDestination(NULL), goalNode(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel)
    : x(x), y(y), vel(vel), nextDestination(NULL), goalNode(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel, double x_goal, double y_goal)
    : x(x), y(y), vel(vel), nextDestination(NULL), world(NULL), E(0.5),
      currNode(NULL), localGoal(NULL)
    {
        goalNode = new Node(new Position(x_goal, y_goal));
        currNode = pathtoGoal.back();
    }

bool Robot::getGoal(double& gx, double & gy){
    if(goalNode == NULL){
        std::cout << "ERR: Goal is empty. Could not get goal" << std::endl;
        return false;
    } 
    gx = goalNode->getPos()->posX;
    gy = goalNode->getPos()->posY;
    return true;
}

void Robot::getRobotPose(double &rx, double &ry){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    rx = x;
    ry = y;
}

Node* Robot::getDestination(){
    std::unique_lock<std::mutex> DestLock(destMtx);
    if(nextDestination == NULL){
        // cout << "WARN: Next destination is queried but found to be NULL. Maybe still planning..?" << endl;
        return NULL;
    }
    return  nextDestination;
}

double Robot::getVel() {return vel;}

void Robot::setWorld(World* world){
    this->world = world;
}

void Robot::setRobotNode(Node* node){
    std::unique_lock<std::mutex> poseLock(this->poseMtx);
    this->currNode = node;
    return;
}

void Robot::setDestination(Node* dest){
    std::unique_lock<std::mutex> DestLock(destMtx);
    this->nextDestination = dest;
}

bool Robot::robotAtGoal(){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    double radialPos = sqrt(pow(currNode->getPos()->posX - goalNode->getPos()->posX,2)+pow(currNode->getPos()->posY - goalNode->getPos()->posY,2));
    if (radialPos < 3){
        return true;
    }
    return false;
}

bool Robot::robotAtDestination(){
    if(nextDestination == NULL){
        cout << "Something is off. Checking robot at destination before making a destination" << endl;
        return false;
    }
    std::unique_lock<std::mutex> poseLock(poseMtx);
    double radialPos = sqrt(pow(currNode->getPos()->posX - nextDestination->getPos()->posX,2)+pow(currNode->getPos()->posY - nextDestination->getPos()->posY,2));
    if (radialPos < 1){
        return true;
    }
    return false;
}

vector<Node*> Robot::getLocalTree(){
    return globalTree;
}

vector<Node*> Robot::getPlan(){
    return pathtoGoal;
}

Node* Robot::getRobotNode(){ return currNode; }
Node* Robot::getReplanGoal(){ return localGoal; }
