
#include <Arduino.h>

/*
 *  BLE Write "password" check
 */

#define kSecurityCodeHighByte 0xF0
#define kSecurityCodeLowByte 0x32

bool weakSecurityCheckPassed(uint8_t data[], uint16_t len);

/*
 * Servo Position BLE Data conversion
 */

typedef class ServoPosition {
  public:
    int desiredPosition;
};

ServoPosition newServoPosition(uint8_t data[], uint16_t len);

/*
 * Motor Position BLE Data conversion
 */

typedef enum MotorDirection {
  UP,
  DOWN,
  STOPPED
};

typedef class MotorPosition {
  public:
    int speed;
    MotorDirection direction;
    bool autostop;
};

MotorPosition newMotorPosition(uint8_t data[], uint16_t len);

/*
 * Battery Voltage BLE Data conversion
 */

typedef class Battery {
  public:
  uint16_t rawSample;
};

/*
 * Robot Position BLE Data conversion
 */

typedef class RobotPosition {

};

