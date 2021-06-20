
#include "vector"
#include "math.h"
#include "random"
#include "map"

class Position{
public:
    int posX;
    int posY;

    Position(int x, int y){
        posX = x;
        posY = y;
    }
};

class Node{
public:

    Position* pos;
    Node* parent;
    std::vector<Node*> children;

    Node(Position* pos){
        this->pos = pos;
        this->parent = NULL;
        return;
    }

    Position* getPos(){
        return this->pos;
    }

    void setPos(Position* pos){
        this->pos = pos;
        return;
    }

    void setParent(Node* parent){
        this->parent = parent;
        return;
    }

    Node* getParent(){
        return this->parent;
    }

    void addChild(Node* child){
        this->children.push_back(child);
    }
};


class Obstacle{
public:

    Position* center;
    int radius;

    Obstacle(Position* center, int radius){
        this->center = center;
        this->radius= radius;
        return;
    }
};

class Tree{
public:

    Node* root;
    std::vector<Node*> treeNs;

    Tree(Node* rootInit){
        root = rootInit;
        root->parent = NULL;
        treeNs.push_back(rootInit);
    }

    void addNode(Node* addEnd, Node* addEr){
        addEnd->children.push_back(addEr);
        addEr->parent = addEnd;
        this->treeNs.push_back(addEr);
        return;
    }

    Node* findNearest(Node* randomNode){
        float minDist = INFINITY;
        Node* nearestNode;
        for (Node* node:this->treeNs){
            float  dist = estDist(node, randomNode);
            if( dist < minDist){
                minDist = dist;
                nearestNode = node;
            }
        }
        return nearestNode;
    }

    float estDist(Node* node, Node* randomNode){

        return sqrt(pow(node->pos->posY - randomNode->pos->posY, 2) + pow(node->pos->posX - randomNode->pos->posX,2));
    }

    Node* createNewNodetoNearest(Node* nearestNode, Node* randomNode){
        int DELTA = 8;
        Node* addedNodeFromSpace;
        if (estDist(nearestNode, randomNode)<DELTA){
            addedNodeFromSpace = randomNode;
        }
        else {
            float theta = atan2(abs(nearestNode->pos->posY-randomNode->pos->posY),abs(nearestNode->pos->posX-randomNode->pos->posX));
            Node* expandTo = new Node(new Position(nearestNode->pos->posX + DELTA * cos(theta), nearestNode->pos->posY + DELTA * sin(theta)));
            addedNodeFromSpace = expandTo;
        }
        return addedNodeFromSpace;
    }

    Node* expandToRandom(Node* expandFrom, Node* randomNode){
        int DELTA = 8;
        Node* addedNodeFromSpace;
        if (estDist(expandFrom, randomNode)<DELTA){
            addNode(expandFrom, randomNode);
            addedNodeFromSpace = randomNode;
        }
        else {
            float theta = atan2(abs(expandFrom->pos->posY-randomNode->pos->posY),abs(expandFrom->pos->posX-randomNode->pos->posX));
            Node* expandTo = new Node(new Position(expandFrom->pos->posX + DELTA * cos(theta), expandFrom->pos->posY + DELTA * sin(theta)));
            addNode(expandFrom, expandTo);
            addedNodeFromSpace = expandTo;
        }
        return addedNodeFromSpace;
    }

};
