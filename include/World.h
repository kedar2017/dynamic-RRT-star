#pragma once
#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>
#include <Robot.h>

class World{
public:
    Robot* robot;

    int updateRatems; // in millisecond  
    
    unsigned long int system_time;
    std::mutex timeMtx;
    
    // ctor
    World(Robot* ptrRobot, int rate);

    // Update function to bind to thread
    void update();

    // update time
    void updateTime();

    // time query
    unsigned long int get_system_time();
};

World::World(Robot* ptrRobot, int rate)
    : robot(ptrRobot), updateRatems(rate), system_time(0) {}

unsigned long int World::get_system_time(){
    std::unique_lock<std::mutex> timeLock(timeMtx);
    return system_time;
}

void World::updateTime(){
    std::unique_lock<std::mutex> timeLock(timeMtx);
    system_time += 1;
}

void World::update(){
    double new_robot_x, new_robot_y;
    double robotVel = robot->getVel();

    while(get_system_time() < 1e20){
        
        // increment sytem time
        updateTime();
        //cout << "Trying World" << endl;
        // Move robot if robot not at destination pose - simple position control const vel
        Node* curr = robot->getRobotNode();
        Node* dest = robot->getDestination();
        if(!robot->robotAtDestination()) // if destination exists
        {   
            // update robot position
            double vdt = robotVel*updateRatems/1000;
            double dist = sqrt(pow(dest->getPos()->posX - curr->getPos()->posY, 2) + pow(dest->getPos()->posY - curr->getPos()->posY, 2));
            if(dist < vdt)
            {
                new_robot_x = dest->getPos()->posX;
                new_robot_y = dest->getPos()->posY;
            }
            else{
                new_robot_x = robot_x + vdt*(dest->getPos()->posX - curr->getPos()->posX)/dist;
                new_robot_y = robot_y + vdt*(dest->getPos()->posY - curr->getPos()->posY)/dist;
            }
            
            robot->setRobotPose(new_robot_x, new_robot_y);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(updateRatems)); // sleep
    }
} 
