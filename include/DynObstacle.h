#ifndef DYNAMOBSTACLE_H
#define DYNAMOBSTACLE_H

#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

class DynamObstacle
{
public:
    int obs_idx;
    bool is_static;
    double sx, sy; // obstacle center position, start
    double ex, ey; // obstacle center position, end
    double path_steps;
    pair<double, double> velocity; // next_pos = (x+velocity.first, y+velocity.second)

    bool is_circle; // obstacle type (true:circle, false:rectangle;)
    double radius; // radius for circle only
    double length; // length for rectangle only
    double width; // width for rectangle only
    double theta; // theta for rectangle only


    // constructors
    DynamObstacle();
    DynamObstacle(int obs_idx, bool is_static, double sX, double sY, double radius, double eX, double eY, double path_steps);

    // check if this obstacle is static
    bool isStatic();

    // check if this obstacle is circle
    bool isCircle();

    // check if a lane formed by 2 points collision free
    bool isLaneCollisionFree(double x1, double y1, double x2, double y2, double curr_step);

    // check if a point is collision free with this obstacle
    bool isStaticValid(double x1, double y1, double curr_step);

    // check if a point is collision free with an obstacle interpolation
    bool isDynamicValid(double x1, double y1, double curr_step, double interp_step);

    // check if an edge is collision free with an obstacle interpolation
    bool isEdgeDynamicValid(double x1, double y1, double x2, double y2, double curr_step, double interp_step);

    // compute distance of obstacle center to a lane formed by 2 points
    double dist2Lane(double x1, double y1, double x2, double y2, double curr_step);

    // tell if the obstacle is moving from the start or from the end
    pair<bool, double> get_velocity_mode(double curr_step);

    // get current position of the obstacle
    pair<double, double> get_position(double curr_step);

    // get the properties of the obstacle
    vector<double> get_obs_feature(double curr_step);

    bool radialPosCheckFloat(Position* checkPoint, Position* center, int radius);

    bool insidePolygon(Position* point, Position* robotPos);

    double dotProduct(Position vec1,Position vec2);

    Position crossProduct(Position vec1,Position vec2);

    double lambdaCalc(Position* start,Position* end,Position* obstacle);

    double distToLine(Position* robotPos, Position* start, Position* end);

    bool linePassesRobot(Position* start, Position* end, Position* robotPos);
};

// constructors
DynamObstacle::DynamObstacle() {};
DynamObstacle::DynamObstacle(int obs_idx, bool is_static, double sX, double sY, double radius, double eX, double eY, double path_steps) {
    // initialize an obstacle class
    this->obs_idx = obs_idx;
    this->is_static = is_static;
    this->sx = sX;
    this->sy = sY;
    this->is_circle = true;
    this->radius = radius;

    // dynamics
    this->ex = eX;
    this->ey = eY;
    this->path_steps = path_steps;

    // compute velocity
    this->velocity = make_pair((eX - sX) / path_steps, (eY - sY) / path_steps);
}

bool DynamObstacle::isStatic() {
    return this->is_static;
}

bool DynamObstacle::isCircle() {
    return this->is_circle;
}

bool DynamObstacle::isLaneCollisionFree(double x1, double y1, double x2, double y2, double curr_step) {
    // check if this obstacle is free from collision with an edge
    // (x1, y1): lane segment start position
    // (x2, y2): lane segment end position
    double dist = dist2Lane(x1, y1, x2, y2, curr_step);
    if (this->radius >= dist) {
        return false;
    }
    else {
        return true;
    }
}

bool DynamObstacle::isStaticValid(double x1, double y1, double curr_step) {
    // check if this obstacle is free from collision with a point

    // extract current position
    pair<double, double> pos_curr = get_position(curr_step);
    double x_curr = pos_curr.first;
    double y_curr = pos_curr.second;

    double dist = sqrt((x1 - x_curr) * (x1 - x_curr) + (y1 - y_curr) * (y1 - y_curr));
    if (this->is_circle) { // circle obstacle
        if (this->radius >= dist) {
            return false;
        }
        else {
            return true;
        }
    }
    else { // rectangular obstacle
        double A = tan(this->theta);
        double B = -1;
        double C = y_curr - x_curr * tan(this->theta);
        double dist1 = abs(A * x1 + B * y1 + C) / sqrt(A * A + B * B);
        double dist2 = sqrt(dist * dist - dist1 * dist1);

        if (dist1 > this->width / 2 || dist2 > this->length / 2) {
            return true;
        }
        else {
            return false;
        }
    }
}

bool DynamObstacle::isDynamicValid(double x1, double y1, double curr_step, double interp_step) {

    // check if this obstacle extension is free from collision with a point

    // extract current position
    pair<double, double> pos_curr = get_position(curr_step);
    double x_curr = pos_curr.first;
    double y_curr = pos_curr.second;
    // compute the next position
    // 1. compute the unit velocity
    pair<double, double> pos_delta = get_position(curr_step + 1);
    double x_delta = pos_delta.first;
    double y_delta = pos_delta.second;
    double vx = x_delta - x_curr;
    double vy = y_delta - y_curr;

    // 2. compute the next position with interpolated unit velocity
    double x_next = vx * interp_step + x_curr;
    double y_next = vy * interp_step + y_curr;

    if(linePassesRobot(new Position(x_curr,y_curr), new Position(x_next,y_next), new Position(x1,y1))){

        return false;
    }

    /*
    double dist = sqrt((x_next - x_curr) * (x_next - x_curr) + (y_next - y_curr) * (y_next - y_curr));
    double dist_curr = sqrt((x1 - x_curr) * (x1 - x_curr) + (y1 - y_curr) * (y1 - y_curr));
    double dist_next = sqrt((x1 - x_next) * (x1 - x_next) + (y1 - y_next) * (y1 - y_next));
        // point enclosed by either circle
    if (this->radius >= dist_curr || this->radius >= dist_next) {
        std::cout<<"Smallerr than circle???"<<std::endl;
        return false;
    }

    // compute two cosine at two endpoints
    double cos_gn = (dist_next * dist_next + dist * dist - dist_curr * dist_curr) / (2 * dist_next * dist);
    double cos_gc = (dist_curr * dist_curr + dist * dist - dist_next * dist_next) / (2 * dist_curr * dist);
    // if either cos is greater than 90 degree => outside => valid
    if (cos_gc <= 0 || cos_gn <= 0) {
        return true;
    }

    // the point is in between of the two endpoints, check vertical distance
    double sin_gc = sqrt(1.0 - cos_gc * cos_gc);
    double dist_vert = dist_curr * sin_gc;
    if (this->radius >= dist_vert) {

        std::cout<<cos_gn<<std::endl;
        std::cout<<cos_gc<<std::endl;
        std::cout<<"Vertical distance problem????"<<std::endl;
        return false;
    }
    */

    return true;
}

bool DynamObstacle::isEdgeDynamicValid(double x1, double y1, double x2, double y2, double curr_step, double interp_step) {
    // inputs note: two endpoints of the edge (x1, y1), (x2, y2)
    // separately check 10 points in the middle

    // directional gradient
    double dx = (x2 - x1) / 10;
    double dy = (y2 - y1) / 10;

    // sequentially check each point
    for (int i = 0; i <= 10; ++i) {
        if (!isDynamicValid(x1 + dx * i, y1 + dy * i, curr_step, interp_step)) {
            return false;
        }
    }
    return true;
}

double DynamObstacle::dist2Lane(double x1, double y1, double x2, double y2, double curr_step) {
    // compute coeff for lane segment Ax+By+C=0
    double A = y1 - y2;
    double B = x2 - x1;
    double C = x1 * y2 - x2 * y1;

    // extract current position
    pair<double, double> pos_curr = get_position(curr_step);
    double x_curr = pos_curr.first;
    double y_curr = pos_curr.second;
    // compute distance of obstacle center to lane
    double dist = abs(A * x_curr + B * y_curr + C) / sqrt(A * A + B * B);

    return dist;
}

// tell if the obstacle is moving from the start or from the end
pair<bool, double> DynamObstacle::get_velocity_mode(double curr_step) {
    pair<bool, double> res;

    double div = curr_step / path_steps;
    int step_rounds = floor(div);

    // check if the velocity mode is forward
    bool forward = false;
    if (step_rounds % 2 == 0) {
        forward = true;
    }

    // compute the applied time steps for current velocity mode
    double applied_steps = curr_step - path_steps * (double)step_rounds;

    res = make_pair(forward, applied_steps);

    return res;
}

// get current position of the obstacle
pair<double, double> DynamObstacle::get_position(double curr_step) {
    pair<bool, double> velocity_mode = get_velocity_mode(curr_step);
    bool forward = velocity_mode.first;
    double applied_steps = velocity_mode.second;

    double x_curr, y_curr;
    if (forward) {
        x_curr = sx + applied_steps * velocity.first;
        y_curr = sy + applied_steps * velocity.second;
    }
    else {
        x_curr = ex - applied_steps * velocity.first;
        y_curr = ey - applied_steps * velocity.second;
    }

    pair<double, double> pos_curr = make_pair(x_curr, y_curr);

    return pos_curr;
}

vector<double> DynamObstacle::get_obs_feature(double curr_step) {
    // get the properties of the obstacle
    // for circular obstacles:      {x, y, radius}
    // for rectangular obstacles:   {x, y, length, width, theta}
    vector<double> obs_features;

    // position
    pair<double, double> pos_curr = get_position(curr_step);
    obs_features.push_back(pos_curr.first);
    obs_features.push_back(pos_curr.second);

    if (is_circle) {
        // size
        obs_features.push_back(radius);
    }
    else {
        // size
        obs_features.push_back(length);
        obs_features.push_back(width);
        obs_features.push_back(theta);
    }

    return obs_features;
}

bool DynamObstacle::radialPosCheckFloat(Position* checkPoint, Position* center, int radius){
    double radialPos = sqrt(pow(checkPoint->posX -center->posX,2)+pow(checkPoint->posY -center->posY,2));
    if (radialPos <= radius){

        std::cout<<"RAdial pos check FAILED !!!"<<std::endl;
        return true;
    }
    return false;
}

bool DynamObstacle::insidePolygon(Position* point, Position* robotPos){
    if (radialPosCheckFloat(point, robotPos, 2+3)){
        return true;
    }
    return false;
}

double DynamObstacle::dotProduct(Position vec1,Position vec2){
    return vec1.posX*vec2.posX+vec1.posY*vec2.posY;
}

Position DynamObstacle::crossProduct(Position vec1,Position vec2){
    return Position(0,vec1.posX*vec2.posY - vec1.posY*vec2.posX);
}

double DynamObstacle::lambdaCalc(Position* start,Position* end,Position* obstacle){
    double nume = dotProduct(Position(end->posX-obstacle->posX,end->posY-obstacle->posY), Position(end->posX-start->posX,end->posY-start->posY));
    double deno = dotProduct(Position(end->posX-start->posX,end->posY-start->posY),Position(end->posX-start->posX,end->posY-start->posY));
    if(deno==0){
        std::cout<<end->posX<<std::endl;
        std::cout<<start->posX<<std::endl;
        std:cout<<"WAZOOOOOOOOO!!!!"<<std::endl;
    }
    return nume/deno;
}

double DynamObstacle::distToLine(Position* robotPos, Position* start, Position* end){
    double x0 = robotPos->posX;
    double x1 = start->posX;
    double x2 = end->posX;
    double y0 = robotPos->posY;
    double y1 = start->posY;
    double y2 = end->posY;
    double nume = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1));
    double deno = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
    return nume/deno;
}

bool DynamObstacle::linePassesRobot(Position* start, Position* end, Position* robotPos){

    if (insidePolygon(start,robotPos) || insidePolygon(end,robotPos))
        return true;
    
    if (distToLine(robotPos,start,end) < 2+3){
        int lambd = lambdaCalc(start,end,robotPos);
        Position pointOnLine = Position(lambd*start->posX+(1-lambd)*end->posX,lambd*start->posY+(1-lambd)*end->posY);
        Position vecComm = Position(pointOnLine.posX-robotPos->posX,pointOnLine.posY-robotPos->posY);
        Position vecDiffA= Position(start->posX-robotPos->posX,start->posY-robotPos->posY);
        Position vecDiffB= Position(end->posX-robotPos->posX,end->posY-robotPos->posY);
        if (dotProduct(crossProduct(vecDiffA,vecComm),crossProduct(vecDiffB,vecComm))>0){
            return false;
        }
        else{
            std::cout<<distToLine(robotPos,start,end)<<std::endl;
            std::cout<<vecComm.posX<<std::endl;
            std::cout<<vecDiffA.posX<<std::endl;
            std::cout<<vecDiffB.posX<<std::endl;
            std::cout<<crossProduct(vecDiffA,vecComm).posY<<std::endl;
            std::cout<<crossProduct(vecDiffB,vecComm).posY<<std::endl;
            std::cout<<dotProduct(crossProduct(vecDiffA,vecComm),crossProduct(vecDiffB,vecComm))<<std::endl;
            std::cout<<"Line passes robot check FAILED !!!"<<std::endl;
            return true;
        }
    }
    return false;
}


#endif