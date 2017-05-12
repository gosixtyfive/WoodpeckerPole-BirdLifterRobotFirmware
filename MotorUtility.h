
#include "RobotBLE.h"

class MotorController {
  int enable_pin;
  int direction_a_pin;
  int direction_b_pin;
  
  int time_to_run_ms;
  int timer_ticks_ms;
  bool isAtUpperLimit;
  bool isAtLowerLimit;
  bool isGoingUp;
  bool isGoingDown;
  bool isAutostopActive;
  
  public:
  MotorController(int enable, int dir_a, int dir_b);
  void executeCommand(MotorPosition motorCommand);
  void stop();
  void update();
  void setAtUpperLimit(bool isATLimit);
  void setAtLowerLimit(bool isATLimit);
};


