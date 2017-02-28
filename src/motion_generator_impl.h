#include <franka/motion_generator.h>
#include <research_interface/rbk_types.h>
#include "robot_impl.h"

namespace franka {

class MotionGenerator {
 public:
  explicit MotionGenerator(franka::Robot::Impl& robot)
      : command_{robot.getRobotCommand().motion},
        motion_generator_running_{robot.getMotionGeneratorRunning()} {
    if (motion_generator_running_) {
      throw MotionGeneratorException(
          "libfranka:: Attempt to start multiple motion generators!");
    }
    motion_generator_running_ = true;
  }

  MotionGenerator(MotionGenerator&& impl) noexcept
      : command_{impl.command_},
        motion_generator_running_{impl.motion_generator_running_} {}

  virtual ~MotionGenerator() noexcept { motion_generator_running_ = false; }

  MotionGenerator() = delete;
  MotionGenerator(const MotionGenerator&) = delete;
  MotionGenerator& operator=(const MotionGenerator&) = delete;
  MotionGenerator& operator=(const MotionGenerator&&) = delete;

  research_interface::MotionGeneratorCommand& command() { return command_; }

 private:
  research_interface::MotionGeneratorCommand& command_;
  bool& motion_generator_running_;
};

class CartesianPoseMotionGenerator::Impl : public MotionGenerator {
 public:
  using MotionGenerator::MotionGenerator;
};

class CartesianVelocityMotionGenerator::Impl : public MotionGenerator {
 public:
  using MotionGenerator::MotionGenerator;
};

class JointPoseMotionGenerator::Impl : public MotionGenerator {
 public:
  using MotionGenerator::MotionGenerator;
};

class JointVelocityMotionGenerator::Impl : public MotionGenerator {
 public:
  using MotionGenerator::MotionGenerator;
};

}  // namespace franka
