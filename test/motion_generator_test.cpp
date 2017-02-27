#include <franka/motion_generator.h>
#include <franka/robot.h>
#include <iostream>
#include <franka/exception.h>
#include <unistd.h>

int main(){

std::cout << "connecting to robot" << std::endl;
try {
franka::Robot robot("127.0.0.1");
std::cout << "Starting Motion Generator" << std::endl;
franka::CartesianPoseMotionGenerator motion_generator = std::move(robot.startCartesianPoseMotionGenerator());
std::cout << "succeeded to start motion generator! " << std::endl;
}
catch (franka::NetworkException const& e)
{
   std::cout<< e.what() << std::endl;
}
usleep(1000000);

return 0;
}
