#pragma once
#include "vector"
#include "math.h"
#include "random"
#include "map"

#include "Parameters.h"

class Position{
public:
    double posX;
    double posY;

    Position(double x, double y){
        posX = x;
        posY = y;
    }
};

class Node{
public:

    Position* pos;
    double cost;
    Node* parent;
    std::vector<Node*> children;

    Node(Position* pos){
        this->pos = pos;
        this->cost= 0.000;
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

    double getCost(){
        return this->cost;
    }

    void setCost(double cost){
        this->cost=cost;
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
    double radius;

    Obstacle(Position* center, double radius){
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

    double estDist(Node* node, Node* randomNode){
        return sqrt(pow(node->pos->posY - randomNode->pos->posY, 2) + pow(node->pos->posX - randomNode->pos->posX,2));
    }

    void addNode(Node* addEnd, Node* addEr){
        double newCost = estDist(addEnd,addEr);
        addEr->setCost(newCost+addEnd->getCost());
        addEnd->children.push_back(addEr);
        addEr->parent = addEnd;
        this->treeNs.push_back(addEr);
        return;
    }

    Node* findNearest(Node* randomNode){
        double minDist = INFINITY;
        Node* nearestNode;
        for (Node* node:this->treeNs){
            double  dist = estDist(node, randomNode);
            if( dist < minDist){
                minDist = dist;
                nearestNode = node;
            }
        }
        return nearestNode;
    }

    Node* createNewNodetoNearest(Node* nearestNode, Node* randomNode){
        //double DELTA = 10.000;
        Node* addedNodeFromSpace;
        if (estDist(nearestNode, randomNode)<DELTA){
            addedNodeFromSpace = randomNode;
        }
        else {
            double theta = atan2(abs(nearestNode->pos->posY-randomNode->pos->posY),abs(nearestNode->pos->posX-randomNode->pos->posX));
            Node* expandTo = new Node(new Position(nearestNode->pos->posX + DELTA * cos(theta), nearestNode->pos->posY + DELTA * sin(theta)));
            addedNodeFromSpace = expandTo;
        }
        return addedNodeFromSpace;
    }

    Node* expandToRandom(Node* expandFrom, Node* randomNode){
        //double DELTA = 10.000;
        Node* addedNodeFromSpace;
        if (estDist(expandFrom, randomNode)<DELTA){
            addNode(expandFrom, randomNode);
            addedNodeFromSpace = randomNode;
        }
        else {
            double theta = atan2(abs(expandFrom->pos->posY-randomNode->pos->posY),abs(expandFrom->pos->posX-randomNode->pos->posX));
            Node* expandTo = new Node(new Position(expandFrom->pos->posX + DELTA * cos(theta), expandFrom->pos->posY + DELTA * sin(theta)));
            addNode(expandFrom, expandTo);
            addedNodeFromSpace = expandTo;
        }
        return addedNodeFromSpace;
    }

    std::vector<Node*> findNodesinRadius(Node* randomNode){
        //double ballRad = 6.000;
        std::vector<Node*> nearQ;
        for (Node* node:this->treeNs){
            double  dist = estDist(node, randomNode);
            if( dist < ballRad){
                nearQ.push_back(node);
            }
        }
        return nearQ;
    }

};
