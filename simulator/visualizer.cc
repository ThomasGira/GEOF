#include <stdio.h>
#include <opencv2/opencv.hpp>


namespace geoff {
namespace viz{
    void DisplayImage(cv:: Mat image, std::string window_name){
        cv::imshow(window_name, image);
        cv::waitKey(1);
    }
    void CreateWindow(std::string window_name){
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE );
    }

    cv::Mat LoadImage(std::string file_path){
        cv::Mat image = cv::imread( file_path, 1 );
        if ( !image.data ){
            printf("No image data \n");
        }
        cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);
        cv::threshold(image,image,100,255,cv::THRESH_BINARY);
        return image;
    }

    cv::Mat RotateImage(cv::Mat src, float angle){
        // get rotation matrix for rotating the image around its center in pixel coordinates
        cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
        cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
        // determine bounding rectangle, center not relevant
        cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
        // adjust transformation matrix
        rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
        rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

        cv::Mat dst;
        cv::warpAffine(src,dst, rot, 
                bbox.size(),
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT,
                cv::Scalar(255,255,255));
        
        return dst;
    }

    cv::Mat PlaceObject(cv::Mat map, cv::Mat obj, int x, int y, float angle){
        cv::Mat rot_obj = RotateImage(obj,angle);
        int left_shift = (int) rot_obj.rows / 2.0;
        int down_shift = (int) rot_obj.cols / 2.0;
        rot_obj.copyTo(map(cv::Rect(x - left_shift,y - down_shift,rot_obj.cols, rot_obj.rows)));

        return map;
    }

}
}