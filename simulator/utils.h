#pragma once

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace geoff{
namespace sim{

bool check_collision(cv::Mat img_in, cv::Point pts[4]);
void disp_mask(cv::Mat img_in, cv::Point pts[4]);
void draw_on(cv::Mat img_in, cv::Point pts[4]);

}
}