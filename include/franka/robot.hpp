#ifndef FRANKA_ROBOT_H_
#define FRANKA_ROBOT_H_

#include <memory>
#include <string>

/**
 * TODO
 */
using RobotState = int;

namespace franka {

/**
 * TODO
 */
class Robot {
 public:
  /**
   * TODO
   */
  using Ptr = std::shared_ptr<Robot>;

  /**
   * TODO
   *
   * @param[in] ip TODO
   * @return TODO
   */
  static Ptr connect(const std::string& ip);

  /**
   * Block until new robot state arrives.
   *
   * @return True if a new robot state arrived, false if a timeout occured.
   */
  bool waitForRobotState();

  /**
   * TODO
   *
   * @return TODO
   */
  RobotState getRobotState() const;

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

 private:
  Robot() = default;
};

}  // namespace franka

#endif  // FRANKA_ROBOT_H_
