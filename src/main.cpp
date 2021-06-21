
#include <iostream>
#include "fstream"
#include "string"
#include <Space.h>
#include <Display.h>

//From Geometry. Need to modify 
bool radialPosCheck(Position* checkPoint, Position* center, int* radius){
    double radialPos = sqrt(pow(checkPoint->posX -center->posX,2)+pow(checkPoint->posY -center->posY,2));
    if (radialPos <= *radius){
        return false;
    }
    return true;
}

bool radialPosCheckFloat(Position* checkPoint, Position* center, int radius){
    double radialPos = sqrt(pow(checkPoint->posX -center->posX,2)+pow(checkPoint->posY -center->posY,2));
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

double lambdaCalc(Position* start,Position* end,Position* obstacle){
    double nume = dotProduct(Position(end->posX-obstacle->posX,end->posY-obstacle->posY), Position(end->posX-start->posX,end->posY-start->posY));
    double deno = dotProduct(Position(end->posX-start->posX,end->posY-start->posY),Position(end->posX-start->posX,end->posY-start->posY));
    return nume/deno;
}

double distToLine(Obstacle* obs, Position* start, Position* end){
    double x0 = obs->center->posX;
    double x1 = start->posX;
    double x2 = end->posX;
    double y0 = obs->center->posY;
    double y1 = start->posY;
    double y2 = end->posY;
    double nume = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1));
    double deno = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
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

Node* chooseParent(Space* space, Tree* tree, Node* randomNode, Node* nearestNode){
    Node* qMin = nearestNode;
    double cMin = nearestNode->getCost() + tree->estDist(nearestNode,randomNode);
    std::vector<Node*> nearestQ = tree->findNodesinRadius(randomNode);
    std::vector<Node*>::iterator it;
    for(it = nearestQ.begin(); it != nearestQ.end(); it++){
        if(checkLineCollision(space, (*it)->getPos(), randomNode->getPos())){
            continue;
        }
        double cNew = (*it)->getCost() + tree->estDist((*it),randomNode);
        if(cNew < cMin){
            cMin = cNew;
            qMin = (*it);
        }
    }
    return qMin;
}

Tree* reWire(Space* space, Tree* tree, Node* randomNode){
    std::vector<Node*> nearestQ = tree->findNodesinRadius(randomNode);
    std::vector<Node*>::iterator it;
    for(it = nearestQ.begin(); it != nearestQ.end(); it++){
        if(checkLineCollision(space, (*it)->getPos(), randomNode->getPos())){
            continue;
        }
        double cNew = (randomNode)->getCost() + tree->estDist((*it),randomNode);
        if(cNew < (*it)->getCost()){
            (*it)->parent->children.erase(std::remove((*it)->parent->children.begin(), (*it)->parent->children.end(), (*it)), (*it)->parent->children.end());
            (*it)->parent = randomNode;
            tree->addNode(randomNode,(*it));
        }
    }
    return tree;
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

std::vector<Node*> generatePath(Tree* tree, Node* start, Node* end){
    Node* parent = NULL;
    parent = end;
    std::vector<Node*> pathToGoal;
    while (parent != start){
        pathToGoal.push_back(parent);
        parent = parent->parent;
    }
    return pathToGoal;
}

void run(Space* space){

    Position* pos = new Position(space->start[0],space->start[1]);
    Node* root = new Node(pos);
    Tree* tree = new Tree(root);
    space->removeNodeFreeSpace(root);
    int threshold = 6;
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
        Node* qMin = chooseParent(space,tree,removeNodefromSpace,nearestNode);
        removeNodefromSpace = expandTree(tree,qMin,removeNodefromSpace);
        updateFreeSpace(space,removeNodefromSpace);
        reWire(space,tree,removeNodefromSpace);
    }
    std::vector<Node*> pathtoGoal=generatePath(tree,root, goalNode);
    printTree(tree);
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    int winX = 100;
    int winY = 100;

    std::vector<int> start{70,10};
    std::vector<int> goal{10,70};
    std::vector<std::vector<int>> obstacles{{15,60,60},{15,17,40},{20,50,80},{30,80,40}};
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
