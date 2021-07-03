#pragma once

#include <iostream>
#include <mutex>
#include </usr/local/Cellar/opencv/4.5.2_4/include/opencv4/opencv2/opencv.hpp>
#include <Space.h>
#include <World.h>

//cv::Mat image(500,500, CV_8UC3, cv::Scalar(0,0,0));

using namespace std;

class Render
{
    private:
    cv::Mat background;
    cv::Mat curScene;
    int refresh_rate;               // viewer refresh rate in ms

    Space* bigSpace;
    Universe* bigUniverse;
    World* bigWorld;
    Robot* robot;
    int flag = 0;

    double map_width, map_height;
    const float scale_factor = 10.0;
    
    bool closeViewer;               // can be fired from main to stop viewer
    std::mutex closeViewerMutex;    // safeguards thread shared variable closeViewer
    std::mutex showViewerMutex;
    // ctor for viewer
    public:
    Render(Space* bigSpace, Universe* bigUniverse, World* bigWorld, Robot* robot, cv::Mat& background);

    // main function to display env - Bind to another thread
    void run();

    // utility drawing functions
    void setFlag(int statVsDyn);
    void DrawObstacle(vector<double>& features, cv::Mat& scene);
    void DrawRobot(cv::Mat& scene);
    void DrawGoal(Space* space,cv::Mat& scene);
    void DrawStart(Space* space,cv::Mat& scene);
    void DrawTree(cv::Mat& scene);
    void DrawPlan(const vector<Node*>& plan, cv::Mat& scene);

    // utils to get, set shared vars
    bool requestViewerClosure();
    bool isViewerClosed();
    cv::Mat getBackground();
};

Render::Render(Space* ptrSpace, Universe* ptrUni, World* ptrWorld, Robot* ptrRobot,cv::Mat& scene)
    : bigSpace(ptrSpace), closeViewer(false), refresh_rate(10), bigUniverse(ptrUni), bigWorld(ptrWorld), robot(ptrRobot), background(scene)
{
    std::cout<<"Initialized renderer..."<<std::endl;
}

void Render::setFlag(int statVsDyn){
    flag = statVsDyn;
    return;
}

void Render::DrawObstacle(vector<double>& features, cv::Mat& scene){
    cv::Scalar colorCircle2(0,100,0);
    //std::cout<<"Draw Obstacle"<<std::endl;
    if(features.size() == 3)
        cv::circle(scene,cv::Point(features[1],features[2]),5*features[0] , colorCircle2 , cv::FILLED);
    else
        cout << "ERROR: Something is wrong. Obst has "<< features.size() << " features" << endl;
}

void Render::DrawGoal(Space* space,cv::Mat& scene){
    int gx, gy;
    gx = space->start[0];
    gy = space->start[1];
    //std::cout<<"Draw Goal"<<std::endl;
    cv::circle(scene, cv::Point(5*gx, 5*gy), 7, cv::Scalar(0,255,0), cv::FILLED);
    return; 
}

void Render::DrawStart(Space* space,cv::Mat& scene){
    int gx, gy;
    gx = space->goal[0];
    gy = space->goal[1];
    //std::cout<<"Draw Start"<<std::endl;
    cv::circle(scene, cv::Point(5*gx, 5*gy), 7, cv::Scalar(0,255,0), cv::FILLED);
    return;
}

void Render::DrawTree(cv::Mat& scene){

    Node* root = bigSpace->tree->root;
    std::vector<Node*> stack;
    stack.push_back(root);

    //std::cout<<"Draw Tree"<<std::endl;

    while (!stack.empty()) {
        Node* currN = stack.front();
        stack.erase(stack.begin());
        for (Node* child:currN->children) {
            if(flag==1)
                cv::line(scene,cv::Point((currN->pos->posX)*5,(currN->pos->posY)*5),cv::Point((child->pos->posX)*5,(child->pos->posY)*5),cv::Scalar(255, 0, 0),2,cv::LINE_8);
            else
                cv::line(scene,cv::Point((currN->pos->posX)*5,(currN->pos->posY)*5),cv::Point((child->pos->posX)*5,(child->pos->posY)*5),cv::Scalar(255,0,0),2,cv::LINE_8);
            stack.insert(stack.begin(),child);
        }
    }
    return;
}

void Render::DrawPlan(const vector<Node*>& plan, cv::Mat& scene){

    //std::cout<<"Draw Plan"<<std::endl;
    for(int i=0; i < plan.size()-1; ++i){
        cv::line(scene, cv::Point(5*plan[i]->getPos()->posX,5*plan[i]->getPos()->posY), 
                     cv::Point(5*plan[i+1]->getPos()->posX, 5*plan[i+1]->getPos()->posY),
                     cv::Scalar(0,0,255), 4,cv::LINE_8);
    }
}


void Render::DrawRobot(cv::Mat& scene){
    if(robot == NULL) {
        cout << "Robot does not exist. Cannot draw robot." << endl;
        return;
    }
    //std::cout<<"Draw Robot"<<std::endl;
    double x, y;
    robot->getRobotPose(x, y);
    cv::circle(scene, cv::Point(5*x, 5*y), 8, cv::Scalar(106,13,173), cv::FILLED);
    return;
}

void Render::run()
{   
    cout << "Starting renderer .. " << endl;
    
    while(!isViewerClosed()){
        //curScene = background.clone();
        DrawTree(curScene);
        std::vector<Obstacle*> staticObs = bigSpace->obstacles;
        for(Obstacle* curr: staticObs){
            vector<double> features = {curr->radius,5*curr->center->posX,5*curr->center->posY};
            DrawObstacle(features, background);
        }

        std::vector<Obstacle*> dynObs = bigUniverse->allSpaces.back()->dynObstacle;
        for(Obstacle* curr: dynObs){
            vector<double> features = {curr->radius,5*curr->center->posX,5*curr->center->posY};
            DrawObstacle(features, background);
        }

        DrawGoal(bigSpace,background);
        DrawStart(bigSpace,background);
        DrawGoal(bigUniverse->allSpaces.back(),background);
        DrawStart(bigUniverse->allSpaces.back(),background);
        DrawPlan(bigSpace->plan, background);
        DrawPlan(bigUniverse->allSpaces.back()->plan, background);

        DrawRobot(background);
    }
    /*
    vector<DynamObstacle*> dynamic_obs = map->get_dynam_obs_vec();

    while(!isViewerClosed()){
        cv::Mat cur_scene = background.clone();
        
        // Get dynamic obstacles and draw them on canvas
        unsigned long int cur_time = world->get_system_time();
        for(auto& cur_dyn_obst: dynamic_obs){
            // Draw the dynamic obstacle
            vector<double> features = cur_dyn_obst->get_obs_feature(cur_time);
            DrawObstacle(features, cur_scene);
        }

        Node* cur_robot_node = robot->getRobotNode();
        DrawTree(cur_robot_node, cur_scene);

        DrawPlan(robot->getPlan(), cur_scene);

        // draw replan goal
        Node* replan_goal = robot->getReplanGoal();
        if(replan_goal) circle(cur_scene, cv::Point2f((replan_goal->x*scale_factor), (map_height - replan_goal->y)*scale_factor), (int)(0.8*scale_factor), cv::Scalar(100,100,0), CV_FILLED);

        DrawRobot(cur_scene);   // draw the robot

        cv::imshow("Environment display", cur_scene);
        cv::waitKey(refresh_rate);
    }
    */
    return;
}

bool Render::requestViewerClosure(){
    std::unique_lock<std::mutex> closeViewerLock(closeViewerMutex);
    closeViewer = true;
    return true;
}

bool Render::isViewerClosed(){
    std::unique_lock<std::mutex> closeViewerLock(closeViewerMutex);
    return closeViewer;
}

cv::Mat Render::getBackground(){
    std::unique_lock<std::mutex> showViewerLock(showViewerMutex);
    return background;
}
