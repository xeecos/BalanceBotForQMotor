#ifndef ENCODED_MOTOR_H
#define ENCODED_MOTOR_H

#define HOLD 0x40
#define SYNC 0x80
// move state and function
#define CMD_RESET 0x00
#define CMD_STOP 0x00
#define CMD_MOVE_TO 0x01
#define CMD_BREAK 0x02
#define CMD_MOVE_SPD 0x03
//#define CMD_MOVE_SPD_DELAY 0x04
// config function
#define CMD_SET_PID 0x10
#define CMD_SET_HOLD 0x11
#define CMD_SET_POWER 0x12
#define CMD_SET_MODE 0x13
#define CMD_SET_PWM 0x14
#define CMD_SET_RATIO 0x15
// get motor status
#define CMD_GET_PID 0x20
#define CMD_GET_POWER 0x21
#define CMD_GET_POS 0x22
#define CMD_GET_SPEED 0x23
#define CMD_GET_RATIO 0x24

enum
{
  MOTOR_1 = 0x00,
  MOTOR_2,
};

typedef struct
{
  long pos;
  long posLast;
  long posSpeed;
  // position pid
  float p;
  float i;
  float d;
  float s;
  float PTerm;
  float ITerm;
  float DTerm;
  float STerm;
  int speed;      //rpm
  int stopCount;
  // output max pwm
  int power;
  // current output pwm
  int pwm;
  // target position
  long targetPos;
  int targetSpd;
  // state
  unsigned char hold;
  unsigned char state;
  unsigned char mode;
  float ratio;
}EMotor;
#endif
