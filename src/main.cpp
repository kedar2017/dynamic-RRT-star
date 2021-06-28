
#include <iostream>
#include "fstream"
#include "string"
#include <Space.h>
#include <Render.h>

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

void printTree(Tree* tree,int flag){
    Node* root = tree->root;
    std::vector<Node*> stack;
    stack.push_back(root);
    while (!stack.empty()) {
        Node* currN = stack.front();
        stack.erase(stack.begin());
        for (Node* child:currN->children) {
            if(flag==1)
                cv::line(image,cv::Point((currN->pos->posX)*5,(currN->pos->posY)*5),cv::Point((child->pos->posX)*5,(child->pos->posY)*5),cv::Scalar(255, 0, 0),2,cv::LINE_8);
            else
                cv::line(image,cv::Point((currN->pos->posX)*5,(currN->pos->posY)*5),cv::Point((child->pos->posX)*5,(child->pos->posY)*5),cv::Scalar(0, 0, 255),2,cv::LINE_8);
            stack.insert(stack.begin(),child);
        }
    }
}

std::vector<Node*> generatePath(Tree* tree, Node* start, Node* end, int flag){
    Node* parent = NULL;
    parent = end;
    std::vector<Node*> pathToGoal;
    while (parent != start){
        if(flag ==1){
            cv::line(image,cv::Point((parent->pos->posX)*5,(parent->pos->posY)*5),cv::Point((parent->parent->pos->posX)*5,(parent->parent->pos->posY)*5),cv::Scalar(255, 255, 255),8,cv::LINE_8);
        }
        else{
            cv::line(image,cv::Point((parent->pos->posX)*5,(parent->pos->posY)*5),cv::Point((parent->parent->pos->posX)*5,(parent->parent->pos->posY)*5),cv::Scalar(255, 0, 255),4,cv::LINE_8);
        }
        pathToGoal.push_back(parent);
        parent = parent->parent;
    }
    return pathToGoal;
}

void run(Space* space, Tree* tree, Node* root, Node* goalNode, int threshold, Node* minStartNode){
    space->removeNodeFreeSpace(root);
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
        if(iterations==1) minStartNode=qMin;
        updateFreeSpace(space,removeNodefromSpace);
        reWire(space,tree,removeNodefromSpace);
    }
}

Space* RRTStar(int deltaWinX, int deltaWinY, std::vector<Node*> pathtoGoal,int startPathPos, int goalPathPos, int obsPathPos){
    std::vector<int> startNew{int(pathtoGoal[startPathPos]->getPos()->posX),int(pathtoGoal[startPathPos]->getPos()->posY)};
    std::vector<int> goalNew{int(pathtoGoal[goalPathPos]->getPos()->posX),int(pathtoGoal[goalPathPos]->getPos()->posY)};
    int winXNew = std::max(startNew[0],goalNew[0])+deltaWinX;
    int winYNew = std::max(startNew[1],goalNew[1])+deltaWinY;
    int startX = std::min(startNew[0],goalNew[0])-deltaWinX;
    int startY = std::min(startNew[1],goalNew[1])-deltaWinY;
    std::vector<std::vector<int>> obstaclesNew{{15,60,60},{15,17,40},{20,50,80},{30,80,40},{2,int(pathtoGoal[obsPathPos]->getPos()->posX),int(pathtoGoal[obsPathPos]->getPos()->posY)}};
    std::vector<int> dynObstacles{2,int(pathtoGoal[obsPathPos]->getPos()->posX),int(pathtoGoal[obsPathPos]->getPos()->posY)};
    Space* spaceNew = new Space(startX, startY, winXNew, winYNew, startNew, goalNew, obstaclesNew, dynObstacles);
    Position* posNew = new Position(spaceNew->start[0],spaceNew->start[1]);
    Node* goalNodeNew = new Node(new Position(spaceNew->goal[0],spaceNew->goal[1]));
    Node* rootNew = new Node(posNew);
    Tree* treeNew = new Tree(rootNew);
    Node* minStartNode;
    run(spaceNew, treeNew, rootNew, goalNodeNew,3,minStartNode);
    spaceNew->tree = treeNew;
    printTree(treeNew,2);
    spaceNew->plan = generatePath(treeNew,rootNew,goalNodeNew,2);

    cv::Scalar colorCircle2(0,100,0);
    cv::circle(image,cv::Point(5*spaceNew->obstacles[4]->center->posX,5*spaceNew->obstacles[4]->center->posY), 7, colorCircle2, cv::FILLED);
    return spaceNew;
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    //Original RRT for finding path 
    //****************************//
    int winX = 100;
    int winY = 100;
    std::vector<int> start{70,10};
    std::vector<int> goal{10,70};
    std::vector<std::vector<int>> obstacles{{15,60,60},{15,17,40},{20,50,80},{30,80,40}};
    std::vector<int> dynObstacles{0,0,1};
    Space* space = new Space(0,0,winX, winY, start, goal, obstacles,dynObstacles);
    Position* pos = new Position(space->start[0],space->start[1]);
    Node* goalNode = new Node(new Position(space->goal[0],space->goal[1]));
    Node* root = new Node(pos);
    Tree* tree = new Tree(root);
    Node* minStartNode;
    run(space, tree, root, goalNode, threshold, minStartNode);
    space->tree = tree;
    printTree(tree,1);
    space->plan = generatePath(tree,root,goalNode,1);
    //****************************//

    //Code handling dynamic obstacles
    //****************************//

    Space* forObsOne = RRTStar(10,10,space->plan,30,24,27);
    Space* forObsTwo = RRTStar(10,10,space->plan,23,17,20);
    Space* forObsThree = RRTStar(10,10,space->plan,16,10,13);

    //****************************//

    int radiusCircle = 30;
    cv::Scalar colorCircle2(0,100,0);
    cv::circle(image,cv::Point(5*start[0],5*start[1]), radiusCircle, colorCircle2, cv::FILLED);
    cv::circle(image,cv::Point(5*goal[0],5*goal[1]), radiusCircle, colorCircle2, cv::FILLED);
    cv::imshow( "Display window", image );
    cv::waitKey(0);

    Render* test = new Render(space);
    test->setFlag(1);
    test->run();

    Render* forObsOneTest = new Render(forObsOne);
    forObsOneTest->setFlag(2);
    forObsOneTest->run();

    Render* forObsTwoTest = new Render(forObsTwo);
    forObsTwoTest->setFlag(2);
    forObsTwoTest->run();

    Render* forObsThreeTest = new Render(forObsThree);
    forObsThreeTest->setFlag(2);
    forObsThreeTest->run();

    return 0;
}
