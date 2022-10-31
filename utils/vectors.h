#pragma once
#include <math.h>
#include<iostream>
#include<ostream>
#include <opencv2/opencv.hpp>
#define PI 3.14159265

namespace geoff {
namespace common {

/**
 * @class Vector2d
 * @brief Vector2d is a class that contains the position and orientation of an bject in 2d
 */
class Vector2d {
 public:
    float x;
    float y;
    float rho;
    /**
    * @brief Constructor
    */
    Vector2d();
    Vector2d(float x, float y, float rho);

    /**
    * @brief Convert coordinates from the current robot frame into world frame.
    * @param robot_pose The vector representing the robots world cordinates.
    * @return New position vector.
    */
    Vector2d robot2world(Vector2d robot_pose);

    /**
    * @brief Conver a provided coordinate in robot frame into world frame.
    * @param point The pair of ints representing the robot frame coordinates.
    * @return The points in world frame.
    */
    std::pair<int,int> point2world(std::pair<int,int> point);
    

    /**
    * @brief Convert coordinates from the current world frame into robot frame.
    * @param robot_pose The vector representing the robots world cordinates.
    * @return New position vector.
    */
    Vector2d world2robot(Vector2d robot_pose);
    Vector2d add(Vector2d vec);
    Vector2d sub(Vector2d vec);
    void add_eq(Vector2d vec);
    void sub_eq(Vector2d vec);
    void print();
    cv::Point to_cv_point();
};

}  // namespace common
}  // namespace geoff