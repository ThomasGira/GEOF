#include <math.h>
#include<iostream>
#include<ostream>
#include"vectors.h"
#define PI 3.14159265

namespace geoff {
namespace common {

Vector2d::Vector2d(){
    this -> x = 0.0;
    this -> y = 0.0;
    this -> rho = 0.0;

}
Vector2d::Vector2d(float x, float y, float rho) {
    this -> x = x;
    this -> y = y;
    this -> rho = rho;
};

Vector2d Vector2d::robot2world(Vector2d robot_pose) {
    // Extract elements
    float robot_x = robot_pose.x;
    float robot_y = robot_pose.y;
    float robot_rho = robot_pose.rho;
    
    // Calcuate the sin and cosine of the angle relative to the world
    float sin_angle = sin(robot_rho);
    float cos_angle = cos(robot_rho);
    
    // Calculate resultant angle
    float new_angle = robot_rho +  this -> rho;
    if (new_angle > 2*PI) {
        new_angle -= 2*PI;
    } else if ( new_angle < -2*PI) {
        new_angle += 2*PI;
    }

    // calculate resultant pose
    float new_x = robot_x + cos_angle *  this -> x - sin_angle *  this -> y;
    float new_y = robot_y + cos_angle *  this -> y + sin_angle *  this -> x;

    return Vector2d(new_x,new_y,new_angle);
};

std::pair<int,int> Vector2d::point2world(std::pair<int,int> point) {
    // Calcuate the sin and cosine of the angle relative to the world
    float sin_angle = sin(rho);
    float cos_angle = cos(rho);

    // calculate resultant pose
    int point_x = (int) (x + cos_angle *  point.first - sin_angle *  point.second);
    int point_y = (int) (y + sin_angle *  point.first + cos_angle *  point.second);

    return std::pair<int,int>{point_x,point_y};
};
    
Vector2d Vector2d::world2robot(Vector2d robot_pose) {
    // Extract elements
    float robot_x = robot_pose.x;
    float robot_y = robot_pose.y;
    float robot_rho = robot_pose.rho;
    
    //Find relative position
    float rel_x = this -> x - robot_x;
    float rel_y = this -> y - robot_y;
    float rel_rho = this -> rho - robot_rho;

    // Calcuate the sin and cosine of the angle relative to the robot
    float sin_angle = sin(robot_rho);
    float cos_angle = cos(robot_rho);
    
    // Calculate resultant angle
    float new_angle = this -> rho - robot_rho;
    if (new_angle > 2*PI) {
        new_angle -= 2*PI;
    } else if ( new_angle < -2*PI) {
        new_angle += 2*PI;
    }

    // calculate resultant pose
    float new_x = cos_angle *  rel_x + sin_angle *  rel_y;
    float new_y = cos_angle *  rel_y - sin_angle *  rel_x;

    return Vector2d(new_x,new_y,new_angle);
};
    
Vector2d Vector2d::add(Vector2d vec){
    x = this -> x + vec.x;
    y = this -> y + vec.y;
    rho = this -> rho + vec.rho;

    return Vector2d(x,y,rho);
};

Vector2d Vector2d::sub(Vector2d vec){
    x = this -> x - vec.x;
    y = this -> y - vec.y;
    rho = this -> rho - vec.rho;

    return Vector2d(x,y,rho);
};

void Vector2d::add_eq(Vector2d vec){
    this -> x += vec.x;
    this -> y += vec.y;
    this -> rho += vec.rho;
};

void Vector2d::sub_eq(Vector2d vec){
    this -> x -= vec.x;
    this -> y -= vec.y;
    this -> rho -= vec.rho;
};
void Vector2d::print(){

    std::string x_str = std::to_string(this -> x);
    std::string y_str = std::to_string(this -> y);
    std::string rho_str = std::to_string(this ->rho);

    std::cout << "X:  " << x_str << " Y:  " << y_str << " Rho:  " << rho_str << std::endl;
};
cv::Point Vector2d::to_cv_point(){
    return cv::Point((int) this -> x, (int) this ->y);
}

}  // namespace common
}  // namespace geoff