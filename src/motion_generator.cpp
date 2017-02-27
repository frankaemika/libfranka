#include <franka/motion_generator.h>
#include "robot_impl.h"

namespace franka {

MotionGenerator::MotionGenerator(franka::Robot::Impl& robot):
    robot_command_(robot.getRobotCommand()),
    motion_generator_running_(robot.getMotionGeneratorRunning()) {
}

MotionGenerator::MotionGenerator(const MotionGenerator&& motion_generator):
    robot_command_(motion_generator.robot_command_),
    motion_generator_running_(motion_generator.motion_generator_running_) {

}

void CartesianPoseMotionGenerator::setDesiredPose(std::array<double, 7>& desired_pose) {
  if(checkPose(desired_pose)) {
    // set
  } else {
    // errormessage
  }
  // TODO;
}

bool CartesianPoseMotionGenerator::checkPose(std::array<double, 7>& pose) {
  // TODO
  return true;
}

void CartesianVelocityMotionGenerator::setDesiredVelocity(std::array<double, 6>& desired_velocity) {
  // TODO
}


void JointPoseMotionGenerator::setDesiredPose(std::array<double, 7>& desired_pose) {
  if(checkPose(desired_pose)) {
    // set
  } else {
    // errormessage
  }
  // TODO
}

bool JointPoseMotionGenerator::checkPose(std::array<double, 7>& desired_pose) {
    // TODO, check unit quaternion
    return true;
}

void JointVelocityMotionGenerator::setDesiredVelocity(std::array<double, 7>& desired_velocity) {
  // TODO
}

}  // namespace franka
