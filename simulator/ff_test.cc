#include"../utils/vectors.h"
#include"../feedforward/network.h"
#include"car.h"

int main(int argc, char** argv )
{
    for (int i =0; i< 10; i++) {
        cv::Mat raw_map = geoff::viz::LoadImage("../assets/map_1.jpg");
        cv::resize(raw_map, raw_map, cv::Size(1000, 1000), cv::INTER_LINEAR);
        geoff::common::Vector2d pose = geoff::common::Vector2d(120,130,-1);
        geoff::viz::CreateWindow("test");
        geoff::sim::Car car = geoff::sim::Car(pose,raw_map);
        geoff::ff::Network network = geoff::ff::Network();

        int index = 0;
        while (!(car.check_collision(raw_map))&( index < 100)){
            index++;
            car.check_lidar();
            std::vector<float> lidar_hits= car.get_lidar_hits();
            network.determine_outputs(lidar_hits);
            std::vector<float> outputs = network.get_output();
            geoff::common::Vector2d add_pose = geoff::common::Vector2d(outputs[0],0,outputs[1]/10.0);
            // add_pose.print();
            // geoff::common::Vector2d add_pose = geoff::common::Vector2d(0,0,0);
            car.add_pose(add_pose);
            // cv::Mat frame = car.draw();
            // geoff::viz::DisplayImage(frame,"test");
        }

        cv::Mat traversed_area = car.get_traversed_area();
        int score = car.get_score();
        std::cout <<"Score: " << score << std::endl;
    }
    return 0;
}