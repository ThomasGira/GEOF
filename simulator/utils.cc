#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace geoff{
namespace sim{

bool check_collision(cv::Mat img_in, cv::Point pts[4]){
    cv::Mat img = img_in.clone();
    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8U);
    cv::fillConvexPoly( mask, pts, 4, cv::Scalar(255) );
    cv::imshow("mask", mask);
    cv::imshow("img", img);
    cv::bitwise_and(img,mask,img);
    cv::bitwise_not(img,img,mask);
    cv::imshow("img post", img);
    cv::threshold(img,img,200,1,cv::THRESH_BINARY);
    cv::Scalar_<double> cost = cv::sum(img);
    std::cout << "Calculated costr: " << cost << std::endl;
    cv::waitKey(0);
    return cost[0] > 0;
};

}
}