#include"car.h"

namespace geoff{
namespace sim{
Car::Car(){
    this -> pose = geoff::common::Vector2d(0,0,0);
    cv::Mat asset = geoff::viz::LoadImage("../assets/car.jpg");
    cv::resize(asset, asset, cv::Size(30, 30), cv::INTER_LINEAR);
    this -> asset = asset;

}
Car::Car(geoff::common::Vector2d initial_pose, cv::Mat base_map){
    this -> prev_time = get_time();
    this -> pose = initial_pose;
    this -> vel_x = 0;
    this -> vel_theta = 0;
    this -> map = base_map;
    cv::Mat asset = geoff::viz::LoadImage("../assets/car.jpg");
    cv::resize(asset, asset, cv::Size(30, 30), cv::INTER_LINEAR);
    this -> asset = asset;


    // Add lidar
    lidar = geoff::sim::Lidar();
};
void Car::check_lidar() {
    lidar.update_pose(this->pose);
    lidar.check_lidar();
};

void Car::draw_lidar() {
    lidar.draw_lidar();
};
geoff::common::Vector2d Car::get_pose(float ax, float at){
    float dt = this -> get_dt();
    update_velocity(ax,at,dt);
    update_pose(dt);

    return this -> pose;
};

std::chrono::steady_clock::time_point Car::get_time(){
    return std::chrono::steady_clock::now();
};

float Car::get_dt(){
    std::chrono::steady_clock::time_point curr_time = get_time();
    std::chrono::duration<double> diff = curr_time - this -> prev_time;
    float dt = (float) diff.count();
    this -> prev_time = curr_time;
    return dt;
    
};

void Car::update_velocity(float ax, float at, float dt){
    this -> vel_x += ax * dt;
    this -> vel_theta += at * dt;
};

void Car::update_pose(float dt){
    float dx = this -> vel_x * dt;
    float dtheta = this -> vel_theta * dt;

    geoff::common::Vector2d rel_delta = geoff::common::Vector2d(dx,0,dtheta);
    geoff::common::Vector2d world_delta = rel_delta.robot2world(this -> pose);

    this -> pose.add_eq(world_delta);
};

bool Car::check_collision(){
    float w = this -> width/2;
    float l = this -> length/2;
    geoff::common::Vector2d fl = geoff::common::Vector2d(l,-w,0).robot2world(this -> pose);
    geoff::common::Vector2d fr = geoff::common::Vector2d(l,w,0).robot2world(this -> pose);
    geoff::common::Vector2d rl = geoff::common::Vector2d(-l,-w,0).robot2world(this -> pose);
    geoff::common::Vector2d rr = geoff::common::Vector2d(-l,w,0).robot2world(this -> pose);

    // std::cout << "w: " << w << "l: " << l << std::endl;
    // std::cout << "X: " << this->pose.x << "Y: " << this->pose.y << std::endl;
    // std::cout << "X: " << fl.x << "Y: " << fl.y << std::endl;
    // std::cout << "X: " << fr.x << "Y: " << fr.y << std::endl;
    // std::cout << "X: " << rl.x << "Y: " << rl.y << std::endl;
    // std::cout << "X: " << rr.x << "Y: " << rr.y << std::endl;
    cv::Point pts[4] = {
        fl.to_cv_point(),
        fr.to_cv_point(),
        rl.to_cv_point(),
        rr.to_cv_point()
    };
    return geoff::sim::check_collision(this -> map,pts);
}

bool Car::check_collision(cv::Mat map){
    float w = this -> width/2;
    float l = this -> length/2;
    geoff::common::Vector2d fl = geoff::common::Vector2d(l,-w,0).robot2world(this -> pose);
    geoff::common::Vector2d fr = geoff::common::Vector2d(l,w,0).robot2world(this -> pose);
    geoff::common::Vector2d rl = geoff::common::Vector2d(-l,-w,0).robot2world(this -> pose);
    geoff::common::Vector2d rr = geoff::common::Vector2d(-l,w,0).robot2world(this -> pose);

    // std::cout << "w: " << w << "l: " << l << std::endl;
    // std::cout << "X: " << this->pose.x << "Y: " << this->pose.y << std::endl;
    // std::cout << "X: " << fl.x << "Y: " << fl.y << std::endl;
    // std::cout << "X: " << fr.x << "Y: " << fr.y << std::endl;
    // std::cout << "X: " << rl.x << "Y: " << rl.y << std::endl;
    // std::cout << "X: " << rr.x << "Y: " << rr.y << std::endl;
    cv::Point pts[4] = {
        fl.to_cv_point(),
        rl.to_cv_point(),
        rr.to_cv_point(),
        fr.to_cv_point()
    };
    return geoff::sim::check_collision(map,pts);
}
cv::Mat Car::draw(cv::Mat frame){
    // Get position nad orientation.
    int x = (int) this -> pose.x;
    int y = (int) this -> pose.y;
    float angle = this -> pose.rho * 180 / 3.14159;

    //Place the car object.
    frame  = geoff::viz::PlaceObject(frame,this -> asset, x,y,angle);
    // Draw each lidar beam.
    // Get list of beam Mats. Each beam is in the robot frame.
    cv::Mat beams = lidar.get_beam_objs();

    // Place beams in frame. Subtract angle of robot to get in world frame.
    frame = geoff::viz::PlaceObject(frame,beams,x,y,-angle);

    return frame;
}

void Car::set_pose(geoff::common::Vector2d pose){
    this -> pose = pose;
}

void Car::add_pose(geoff::common::Vector2d pose){
    geoff::common::Vector2d temp_pose = this -> pose;
    this -> pose = this -> pose.add(pose);
    if ((this -> check_collision())){
        std::cout << "Error: Collision. Stopping movement" << std::endl;
        this -> pose = temp_pose;
    }
}

}
}