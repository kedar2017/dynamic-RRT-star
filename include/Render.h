#pragma once

#include <iostream>
#include <mutex>
#include </usr/local/Cellar/opencv/4.5.2_4/include/opencv4/opencv2/opencv.hpp>

cv::Mat image(500,500, CV_8UC3, cv::Scalar(0,0,0));

using namespace std;

class Render
{
private:
    cv::Mat background;
    int refresh_rate;               // viewer refresh rate in ms

    Space* bigSpace;
    int flag = 0;

    double map_width, map_height;
    const float scale_factor = 10.0;
    
    bool closeViewer;               // can be fired from main to stop viewer
    std::mutex closeViewerMutex;    // safeguards thread shared variable closeViewer

public:
    // ctor for viewer
    Render(Space* bigSpace);

    // main function to display env - Bind to another thread
    void run();

    // utility drawing functions
    void setFlag(int statVsDyn);
    void DrawObstacle(vector<double>& features, cv::Mat& scene);
    void DrawRobot(cv::Mat& scene);
    void DrawGoal(cv::Mat& scene);
    void DrawStart(cv::Mat& scene);
    void DrawTree(cv::Mat& scene);
    void DrawPlan(const vector<Node*>& plan, cv::Mat& scene);

    // utils to get, set shared vars
    bool requestViewerClosure();
    bool isViewerClosed();
};

Render::Render(Space* ptrSpace)
    : bigSpace(ptrSpace), closeViewer(false), refresh_rate(10)
{
    std::cout<<"Initialized renderer..."<<std::endl;
}

void Render::setFlag(int statVsDyn){
    flag = statVsDyn;
    return;
}

void Render::DrawObstacle(vector<double>& features, cv::Mat& scene){
    cv::Scalar colorCircle2(0,100,0);
    if(features.size() == 3)
        cv::circle(scene,cv::Point(features[1],features[2]),features[0] , colorCircle2 , cv::FILLED);
    else
        cout << "ERROR: Something is wrong. Obst has "<< features.size() << " features" << endl;
}

void Render::DrawGoal(cv::Mat& scene){
    int gx, gy;
    gx = bigSpace->start[0];
    gy = bigSpace->start[1];
    cv::circle(scene, cv::Point(5*gx, 5*gy), 7, cv::Scalar(0,255,0), cv::FILLED);
    return; 
}

void Render::DrawStart(cv::Mat& scene){
    int gx, gy;
    gx = bigSpace->goal[0];
    gy = bigSpace->goal[1];
    cv::circle(scene, cv::Point(5*gx, 5*gy), 7, cv::Scalar(0,255,0), cv::FILLED);
    return;
}

void Render::DrawTree(cv::Mat& scene){

    Node* root = bigSpace->tree->root;
    std::vector<Node*> stack;
    stack.push_back(root);
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
    for(int i=0; i < plan.size()-1; ++i){
        cv::line(scene, cv::Point(5*plan[i]->getPos()->posX,5*plan[i]->getPos()->posY), 
                     cv::Point(5*plan[i+1]->getPos()->posX, 5*plan[i+1]->getPos()->posY),
                     cv::Scalar(0,0,255), 4,cv::LINE_8);
    }
}

/*
void Render::DrawRobot(cv::Mat& scene){
    if(robot == NULL) {
        cout << "Robot does not exist. Cannot draw robot." << endl;
        return;
    }
    double x, y;
    robot->getRobotPose(x, y);
    circle(scene, cv::Point2f((x*scale_factor), (map_height - y)*scale_factor), (int)(0.8*scale_factor), cv::Scalar(255,0,0), CV_FILLED);
}

*/
void Render::run()
{   
    cout << "Starting renderer .. " << endl;

    // Set up white background with map size
    background = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0,0,0));
    
    std::vector<Obstacle*> staticObs = bigSpace->obstacles;
    // Draw static obstacles onto background
    for(Obstacle* curr: staticObs){
        vector<double> features = {curr->radius,5*curr->center->posX,5*curr->center->posY};
        DrawObstacle(features, background);
    }

    DrawGoal(background);
    DrawStart(background);
    DrawTree(background);
    DrawPlan(bigSpace->plan, background);

    cv::imshow("Environment display", background);
    cv::waitKey(0);
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