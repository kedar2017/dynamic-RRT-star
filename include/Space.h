#include "vector"
#include "math.h"
#include "random"
#include "map"
#include <Tree.h>

class Space{
public:

    int winX;
    int winY;
    std::vector<int> start;
    std::vector<int> goal;
    std::vector<std::vector<int> > points;
    std::vector<Obstacle*> obstacles;

    Space(int x, int y, std::vector<int> start, std::vector<int> goal, std::vector<std::vector<int>> obst){
        this->winX = x;
        this->winY = y;
        this->start = start;
        this->goal = goal;

        for (int i = 0; i < winX; ++i) {
            for (int j = 0; j < winY; ++j){
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