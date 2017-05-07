#import "MotorUtility.h"

  MotorController::MotorController(int enable, int dir_a, int dir_b) {
    enable_pin = enable;
    direction_a_pin = dir_a;
    direction_b_pin = dir_b;
    time_to_run_ms = 0;
    timer_ticks_ms = 0;
    isAtUpperLimit = false;
    isAtLowerLimit = false;
    isGoingUp = false;
    isGoingDown = false;
  }

   void MotorController::executeCommand(MotorPosition motorCommand) {
    digitalWrite(enable_pin, LOW);
    switch (motorCommand.direction) {
      case UP:
        if (!isAtUpperLimit) {
          isGoingUp = true;
          isGoingDown = false;
          digitalWrite(direction_a_pin, HIGH);
          digitalWrite(direction_b_pin, LOW);
          digitalWrite(enable_pin, HIGH);
        }
        break;
      case DOWN:
      Serial.println("trying down");
        if (!isAtLowerLimit) {
          isGoingUp = false;
          isGoingDown = true;
          digitalWrite(direction_a_pin, LOW);
          digitalWrite(direction_b_pin, HIGH);
          digitalWrite(enable_pin, HIGH);

        }
        break;
      default:
        digitalWrite(direction_a_pin, LOW);
        digitalWrite(direction_b_pin, LOW);
        digitalWrite(enable_pin, LOW);
        isGoingUp = false;
        isGoingDown = false;
    }
    
    if (motorCommand.autostop) {
      timer_ticks_ms = 0;
      time_to_run_ms = 2000;
    } else {
      time_to_run_ms = 0;
    }
    
  }

  void MotorController::stop() {
    digitalWrite(direction_a_pin, LOW);
    digitalWrite(direction_b_pin, LOW);
    digitalWrite(enable_pin, LOW);
    isGoingUp = false;
    isGoingDown = false;
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
    if (isAtLimit & !isGoingDown) {
      stop();
    }
  }
  
  void MotorController::setAtLowerLimit(bool isAtLimit) {
    isAtLowerLimit = isAtLimit;
    if (isAtLimit & !isGoingUp) {
      stop();
    }
  }
 

