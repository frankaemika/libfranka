# libfranka: FRANKA EMIKA Research Interface C++ library

The Research Interface allows for direct access to the FRANKA EMIKA robot. Using `libfranka`, you can connect to the robot, query its current state and provide your own controllers and motion generators.

When using `libfranka`, a typical program structure will use a loop:

```cpp
#include <franka/robot.h>

// ...

franka::Robot robot(robot_hostname);

while (robot.waitForRobotState()) {
  const franka::RobotState& robot_state = robot.robotState();
  // Send commands
}
```

Samples showcasing different `libfranka` usecases can be found in the `examples` folder.
