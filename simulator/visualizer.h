#pragma once
#include <stdio.h>
#include <opencv2/opencv.hpp>


namespace geoff {
namespace viz{
    void DisplayImage(cv::Mat image, std::string window_name);
    void CreateWindow(std::string window_name);
    cv::Mat LoadImage(std::string file_path);
    cv::Mat RotateImage(cv::Mat image, float angle);
    cv::Mat PlaceObject(cv::Mat map, cv::Mat obj, int x, int y, float angle);
    cv::Mat draw_circle(cv::Mat image, std::vector<std::pair<int, int>> points, int r, cv::Scalar color);
    cv::Mat draw_circle(cv::Mat image, int x, int y, int r, cv::Scalar color);
}
}