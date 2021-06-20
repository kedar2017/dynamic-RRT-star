/*
#include <iostream>
#include <thread>
#include <string>
#include <memory>
#include <unistd.h>
using namespace std;

int main(int argc, char* argv[]){
    cout << "Starting program" << endl;
    return 0;
}
*/

#include <iostream>
#include "fstream"
#include "string"
#include <Space.h>
#include <Display.h>

//From Geometry. Need to modify 
bool radialPosCheck(Position* checkPoint, Position* center, int* radius){
    float radialPos = sqrt(pow(checkPoint->posX -center->posX,2)+pow(checkPoint->posY -center->posY,2));
    if (radialPos <= *radius){
        return false;
    }
    return true;
}

bool radialPosCheckFloat(Position* checkPoint, Position* center, int radius){
    float radialPos = sqrt(pow(checkPoint->posX -center->posX,2)+pow(checkPoint->posY -center->posY,2));
    if (radialPos < radius){
        return true;
    }
    return false;
}

bool insidePolygon(Obstacle* obstacle, Position* point){
    if (radialPosCheckFloat(point, obstacle->center, obstacle->radius)){
        return true;
    }
    return false;
}

int dotProduct(Position vec1,Position vec2){
    return vec1.posX*vec2.posX+vec1.posY*vec2.posY;
}

Position crossProduct(Position vec1,Position vec2){
    return Position(0,vec1.posX*vec2.posY - vec1.posY*vec2.posX);
}

int lambdaCalc(Position* start,Position* end,Position* obstacle){
    int nume = dotProduct(Position(end->posX-obstacle->posX,end->posY-obstacle->posY), Position(end->posX-start->posX,end->posY-start->posY));
    int deno = dotProduct(Position(end->posX-start->posX,end->posY-start->posY),Position(end->posX-start->posX,end->posY-start->posY));
    return nume/deno;
}

int distToLine(Obstacle* obs, Position* start, Position* end){
    int x0 = obs->center->posX;
    int x1 = start->posX;
    int x2 = end->posX;
    int y0 = obs->center->posY;
    int y1 = start->posY;
    int y2 = end->posY;

    int nume = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1));
    int deno = sqrt(pow(x2-x1,2) + pow(y2-y1,2));

    return nume/deno;
}

bool linePassesObstacle(Obstacle* obs, Position* start, Position* end){

    if (insidePolygon(obs,start) || insidePolygon(obs,end))
        return true;
    if (distToLine(obs,start,end) < obs->radius){
        int lambd = lambdaCalc(start,end,obs->center);
        Position pointOnLine = Position(lambd*start->posX+(1-lambd)*end->posX,lambd*start->posY+(1-lambd)*end->posY);
        Position vecComm = Position(pointOnLine.posX-obs->center->posX,pointOnLine.posY-obs->center->posY);
        Position vecDiffA= Position(start->posX-obs->center->posX,start->posY-obs->center->posY);
        Position vecDiffB= Position(end->posX-obs->center->posX,end->posY-obs->center->posY);
        if (dotProduct(crossProduct(vecDiffA,vecComm),crossProduct(vecDiffB,vecComm))>0){
            return false;
        }
        else{
            return true;
        }
    }
    return false;
}

Node* generateRandomNode(Space* space){
    return space->getRandomNode();
}

Node* nearestNodeTree(Tree* tree, Node* sampledNode){
    return tree->findNearest(sampledNode);
}

bool checkGoaltoTree(Tree* tree, Node* goalNode, int* threshold){
    Node* nearestNode = nearestNodeTree(tree,goalNode);
    if (radialPosCheck(goalNode->pos,nearestNode->pos,threshold)) {
        return true;
    }
    tree->expandToRandom(nearestNode,goalNode);
    return false;
}

Node* createNewNodeNearest(Tree* tree, Node* startNode, Node* endNode){
    return tree->createNewNodetoNearest(startNode,endNode);
}
Node* expandTree(Tree* tree, Node* startNode, Node* endNode){
    return tree->expandToRandom(startNode, endNode);
}

void updateFreeSpace(Space* space, Node* randomedNode){
    return space->removeNodeFreeSpace(randomedNode);
}

bool checkLineCollision(Space* space, Position* start, Position* end){

    for (Obstacle* obs:space->obstacles){
        if (linePassesObstacle(obs,start,end)){
            return true;
        }
    }
    return false;
}

bool checkPointCollision(Space* space, Position* point){

    for (Obstacle* obs:space->obstacles){
        if (insidePolygon(obs,point))
            return true;
    }
    return false;
}

void printTree(Tree* tree){
    Node* root = tree->root;
    std::vector<Node*> stack;
    stack.push_back(root);
    while (!stack.empty()) {
        Node* currN = stack.front();
        stack.erase(stack.begin());
        for (Node* child:currN->children) {
            cv::line(image,cv::Point((currN->pos->posX)*5,(currN->pos->posY)*5),cv::Point((child->pos->posX)*5,(child->pos->posY)*5),cv::Scalar(255, 0, 0),2,cv::LINE_8);
            stack.insert(stack.begin(),child);
        }
    }
}

void printJSON(Tree* tree){
    for (auto node: tree->treeNs){
        int posX = node->pos->posX;
        int posY = node->pos->posY;
        std::string key = "{"+std::to_string(posX)+","+std::to_string(posY)+"}";
        std::string value = "";
        for (auto child:node->children){
            value = value + "{"+std::to_string(child->pos->posX)+","+std::to_string(child->pos->posY)+"},";
        }
    }
    return;
}

void printPath(Tree* tree, Node* start, Node* end){
    Node* parent = NULL;
    parent = end;
    while (parent != start){
        int posX = parent->pos->posX;
        int posY = parent->pos->posY;
        std::string value = "{"+std::to_string(posX)+","+std::to_string(posY)+"}";
        std::string key = "";
        parent = parent->parent;
        posX = parent->pos->posX;
        posY = parent->pos->posY;
        key = "{"+std::to_string(posX)+","+std::to_string(posY)+"}";
    }
    return;
}

void run(Space* space){

    Position* pos = new Position(space->start[0],space->start[1]);
    Node* root = new Node(pos);
    Tree* tree = new Tree(root);
    space->removeNodeFreeSpace(root);
    int threshold = 9;
    Node* goalNode = new Node(new Position(space->goal[0],space->goal[1]));
    int iterations = 0;

    cv::Mat image(500,500, CV_8UC3, cv::Scalar(0,0,0));


    while (checkGoaltoTree(tree,goalNode,&threshold)){
        iterations++;
        Node* randomNode = generateRandomNode(space);
        if (checkPointCollision(space,randomNode->getPos())){
            continue;
        }
        Node* nearestNode = nearestNodeTree(tree, randomNode);
        Node* removeNodefromSpace = createNewNodeNearest(tree,nearestNode,randomNode);

        if (checkLineCollision(space, nearestNode->getPos(), removeNodefromSpace->getPos())){
            continue;
        }
        removeNodefromSpace = expandTree(tree,nearestNode,randomNode);
        updateFreeSpace(space,removeNodefromSpace);
    }
    printPath(tree,root, goalNode);
    printTree(tree);
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    int winX = 100;
    int winY = 100;

    std::vector<int> start{70,10};
    std::vector<int> goal{10,70};
    std::vector<std::vector<int>> obstacles{{15,60,60},{15,17,40},{20,50,80}};
    Space* space = new Space(winX, winY, start, goal, obstacles);

    run(space);

    int radiusCircle = 30;
    cv::Scalar colorCircle2(0,100,0);
    cv::circle(image,cv::Point(5*start[0],5*start[1]), radiusCircle, colorCircle2, cv::FILLED);
    cv::circle(image,cv::Point(5*goal[0],5*goal[1]), radiusCircle, colorCircle2, cv::FILLED);
    cv::imshow( "Display window", image );
    cv::waitKey(0);
    return 0;
}
