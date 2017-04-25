#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "RobotBLE.h"

class MotorController {
  Adafruit_DCMotor *motor;
  int time_to_run_ms;
  int timer_ticks_ms;
  bool isAtUpperLimit;
  bool isAtLowerLimit;
  
  public:
  MotorController(Adafruit_DCMotor *motor_in);
  void executeCommand(MotorPosition motorCommand);
  void stop();
  void update();
  void setAtUpperLimit(bool isATLimit);
  void setAtLowerLimit(bool isATLimit);
};


