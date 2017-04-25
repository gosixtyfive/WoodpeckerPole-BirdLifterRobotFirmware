
#include "RobotBLE.h"
/*
 *  BLE Write "password" check
 */

bool weakSecurityCheckPassed(uint8_t data[], uint16_t len) {
  if (len < 2) {
    return false;
  } else {
        if (data[0] == 0x32 && data[1] == 0xF0) {
      return true;
    } else {
      return false;
    }
  }
}

/*
 * Servo Position BLE Data conversion
 */

ServoPosition newServoPosition(uint8_t data[], uint16_t len) {
  ServoPosition servo;
  servo.desiredPosition = (int)(data[2]);
  return servo;
}

MotorPosition newMotorPosition(uint8_t data[], uint16_t len) {
  MotorPosition motor;
  int16_t signedSpeed = (int16_t)((uint16_t)data[3] << 8 | (uint16_t)data[2]);
  if (signedSpeed == 0) {
    motor.speed = 0;
    motor.direction = STOPPED;
  } else {
    motor.speed = abs(signedSpeed);
    if (signedSpeed > 0) {
      motor.direction = UP;
    } else {
      motor.direction = DOWN;
    }
  }
  motor.autostop = (data[4] & 0x01 == 0x01);
  return motor;
}


