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
    double newRobotX, newRobotY;
    double robotX, robotY;
    double destX, destY;
    double robotVel = robot->getVel();
    while(get_system_time() < 1e20){
        updateTime();
        robot->getRobotPose(robotX,robotY);
        if((robotX != destX || robotY != destY) && robot->getDestination(destX,destY)) // if destination exists
        {
            //double vdt = robotVel*updateRatems/1000;
            double vdt = 1;
            double dist = sqrt(pow(destX - robotX, 2) + pow(destY - robotY, 2));

            if(dist < vdt)
            {
                newRobotX = destX;
                newRobotY = destY;
            }
            else{
                newRobotX = robotX + vdt*(destX-robotX)/dist;
                newRobotY = robotY + vdt*(destY-robotY)/dist;
            }
            robot->setRobotPose(newRobotX,newRobotY);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // sleep
    }
} 
