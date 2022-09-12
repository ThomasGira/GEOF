#include"vectors.cc"
#include<iostream>
#include<math.h>
#define PI 3.14159265

int main(int argc, char** argv ) {
    for (float wx=0; wx < 5; wx++){
        for (float wy=0; wy < 5; wy++){
            for (float wrho=-2*PI; wrho < 2*PI; wrho+= PI/3){
                for (float x=0; x < 5; x++){
                    for (float y=0; y < 5; y++){
                        for (float rho=-2*PI; rho < 2*PI; rho+= PI/3){
                            geoff::common::Vector2d object_pose = geoff::common::Vector2d(x,y,rho);
                            geoff::common::Vector2d robot_pose = geoff::common::Vector2d(wx,wy,wrho);
                            geoff::common::Vector2d world_pose = object_pose.robot2world(robot_pose);
                            geoff::common::Vector2d object_pose_2 = world_pose.world2robot(robot_pose);

                            std::cout << "--- Robot ---   ";
                            object_pose.print();
                            std::cout << "--- Trans ---   ";
                            robot_pose.print();
                            std::cout << "--- World ---   ";
                            world_pose.print();
                            std::cout << "--- Robot ---   ";
                            object_pose_2.print();
                            std::cout << std::endl;
                        }
                    }
                }
            }
        }
    }

    return 0;
}