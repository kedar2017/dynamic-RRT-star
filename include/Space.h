#pragma once
#include "vector"
#include "math.h"
#include "random"
#include "map"
#include <Tree.h>
#include <DynObstacle.h>

class Space{
public:

    int winX;
    int winY;
    std::vector<int> start;
    std::vector<int> goal;
    std::vector<std::vector<int> > points;
    std::vector<Obstacle*> obstacles;
    std::vector<DynamObstacle*> dynObstacle;
    Tree* tree;
    std::vector<Node*> plan;
    std::mutex mapMutex;

    Space(int startX,int startY,int x, int y, std::vector<int> start, std::vector<int> goal, std::vector<std::vector<int>> obst){
        this->winX = x;
        this->winY = y;
        this->start = start;
        this->goal = goal;
        std::unique_lock<std::mutex> mtxLoc(mapMutex);

        for (int i = startX; i < winX; ++i) {
            for (int j = startY; j < winY; ++j){
                this->points.push_back({i,j});
            }
        }
        initObstacles(obst);
        return;
    }

    void initObstacles(std::vector<std::vector<int> > tuples){
        for (int i = 0; i < tuples.size(); ++i) {
            Position* centerPos = new Position(tuples[i][1],tuples[i][2]);
            Obstacle* obstacleNew = new Obstacle(centerPos,tuples[i][0]);
            this->obstacles.push_back(obstacleNew);
        }
        return;
    }

    void addDynamicObstacle(DynamObstacle* obs){
        this->dynObstacle.push_back(obs);
        return;
    }

    vector<DynamObstacle*> getDynamObs(){
        std::unique_lock<std::mutex> mtxLoc(mapMutex);
        return this->dynObstacle;
    }

    std::vector<int> getRandomPoint(){
        std::random_device randomDevice;
        std::mt19937  engine{randomDevice()};
        std::uniform_real_distribution<double> dist(0, this->points.size());
        return this->points[dist(engine)];
    }

    Node* getRandomNode(){
        std::vector<int> randomPoint = this->getRandomPoint();
        return new Node(new Position(randomPoint[0], randomPoint[1]));
    }

    void removeNodeFreeSpace(Node* node){
        int posX = node->getPos()->posX;
        int posY = node->getPos()->posY;
        int vecIndex = 0;
        for(auto point:this->points)
        {
            if(point[0] == posX && point[1] == posY)
            {
                this->points.erase(this->points.begin()+vecIndex);
                break;
            }
            vecIndex++;
        }
        return;
    }
};

class Universe{
    public:

    std::vector<Space*> allSpaces;

    Universe(){
        return;
    }

    void addSpace(Space* space){
        allSpaces.push_back(space);
        return;
    }

};
