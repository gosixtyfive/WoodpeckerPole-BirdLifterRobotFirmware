#import "MotorUtility.h"

  MotorController::MotorController(Adafruit_DCMotor *motor_in) {
    motor = motor_in;
    time_to_run_ms = 0;
    timer_ticks_ms = 0;
    isAtUpperLimit = false;
    isAtLowerLimit = false;
  }

   void MotorController::executeCommand(MotorPosition motorCommand) {
    motor->run(RELEASE);
    motor->setSpeed((int)motorCommand.speed); 
    Serial.println((int)motorCommand.speed);
    switch (motorCommand.direction) {
      case UP:
        if (!isAtUpperLimit) {
          motor->run(FORWARD);
        }
        break;
      case DOWN:
      Serial.println("trying down");
        if (!isAtLowerLimit) {
          motor->run(BACKWARD);
        }
        break;
      default:
        motor->run(RELEASE);
    }
    
    if (motorCommand.autostop) {
      timer_ticks_ms = 0;
      time_to_run_ms = 2000;
    } else {
      time_to_run_ms = 0;
    }
    
  }

  void MotorController::stop() {
    motor->run(RELEASE);
    motor->setSpeed(0);
  }

  void MotorController::update() {
    //Serial.println("timer");
    timer_ticks_ms = timer_ticks_ms + 10;
    if (time_to_run_ms != 0 && timer_ticks_ms >= time_to_run_ms) {
      Serial.println("Stopping");
      stop();
      time_to_run_ms = 0;
      timer_ticks_ms = 0;
    }
  }

  void MotorController::setAtUpperLimit(bool isAtLimit) {
    isAtUpperLimit = isAtLimit;
    if (isAtLimit) {
      stop();
    }
  }
  
  void MotorController::setAtLowerLimit(bool isAtLimit) {
    isAtLowerLimit = isAtLimit;
    if (isAtLimit) {
      stop();
    }
  }
 

