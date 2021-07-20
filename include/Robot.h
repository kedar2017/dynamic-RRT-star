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
    double gx, gy;
    Node* currNode;       // current node at which robot is present
    Node* goalNode;
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
    bool getDestination(double &rx, double &ry);
    double getVel();
    std::vector<Node*> getCurrentTree();
    std::vector<Node*> getLocalTree();
    std::vector<Node*> getPlan();
    Node* getRobotNode();
    Node* getReplanGoal();

    // Setters
    void setWorld(World* world);                            // main links robot to world
    void setRobotNode(Node* node);    // world updates robot pose
    void setRobotPose(double x, double y);
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
        this->x = x;
        this->y = y;
        this->gx = x_goal;
        this->gy = y_goal;
        this->goalNode = new Node(new Position(this->gx,this->gy));
    }

bool Robot::getGoal(double& gx, double & gy){
    if(this->goalNode == NULL){
        std::cout << "ERR: Goal is empty. Could not get goal" << std::endl;
        return false;
    } 
    this->gx = this->goalNode->getPos()->posX;
    this->gy = goalNode->getPos()->posY;
    return true;
}

void Robot::getRobotPose(double &rx, double &ry){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    rx = this->x;
    ry = this->y;
}

bool Robot::getDestination(double &rx, double &ry){
    std::unique_lock<std::mutex> DestLock(destMtx);
    if(this->nextDestination == NULL){
        return false;
    }
    rx = this->nextDestination->getPos()->posX;
    ry = this->nextDestination->getPos()->posY;
    return true;
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

void Robot::setRobotPose(double xPos, double yPos){
    std::unique_lock<std::mutex> poseLock(this->poseMtx);
    this->x = xPos;
    this->y = yPos;
    return;
}

void Robot::setDestination(Node* dest){
    std::unique_lock<std::mutex> DestLock(destMtx);
    this->nextDestination = dest;
}

bool Robot::robotAtGoal(){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    double radialPos = sqrt(pow(this->x - this->goalNode->getPos()->posX,2)+pow(this->y - this->goalNode->getPos()->posY,2));
    if (radialPos < 2){
        return true;
    }
    return false;
}

bool Robot::robotAtDestination(){
    if(this->nextDestination == NULL){
        cout << "Something is off. Checking robot at destination before making a destination" << endl;
        return false;
    }
    std::unique_lock<std::mutex> poseLock(poseMtx);
    double radialPos = sqrt(pow(this->x - this->nextDestination->getPos()->posX,2)+pow(this->y - this->nextDestination->getPos()->posY,2));
    if (radialPos < 1){
        return true;
    }
    return false;
}

vector<Node*> Robot::getLocalTree(){
    return this->globalTree;
}

vector<Node*> Robot::getPlan(){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    return this->pathtoGoal;
}

Node* Robot::getRobotNode(){ return this->currNode;}
Node* Robot::getReplanGoal(){ return localGoal; }
