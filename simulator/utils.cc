#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace geoff{
namespace sim{

bool check_collision(cv::Mat img_in, cv::Point pts[4]){
    cv::Mat img = img_in.clone();
    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8U);
    cv::fillConvexPoly( mask, pts, 4, cv::Scalar(255) );
    // cv::imshow("mask", mask);
    // cv::imshow("img", img);
    cv::bitwise_and(img,mask,img);
    cv::bitwise_not(img,img,mask);
    // cv::imshow("img post", img);
    cv::threshold(img,img,200,1,cv::THRESH_BINARY);
    cv::Scalar_<double> cost = cv::sum(img);
    // std::cout << "Calculated costr: " << cost[0]  << std::endl;
    // cv::waitKey(0);

    return cost[0] > 0;
};

void disp_mask(cv::Mat img_in, cv::Point pts[4]){
    cv::Mat img = img_in.clone();
    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8U);
    cv::fillConvexPoly( mask, pts, 4, cv::Scalar(0,0,255) );
    cv::bitwise_not(mask,mask);
    cv::imshow("mask", mask);
    cv::waitKey(1);
};

void draw_on(cv::Mat img_in, cv::Point pts[4]){
    cv::Mat img = img_in.clone();
    cv::fillConvexPoly( img, pts, 4, cv::Scalar(0) );
    cv::imshow("img", img);
    cv::waitKey(1);
};


}
}