#include <Wire.h>
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include "encodedMotor.h"

#define INT_1_PIN 3
#define DIR_1_PIN 10
#define INT_2_PIN 2
#define DIR_2_PIN 12

#define MOTOR_1_PWM 6
#define MOTOR_1_H1 A0
#define MOTOR_1_H2 A1

#define MOTOR_2_PWM 5
#define MOTOR_2_H1 A2
#define MOTOR_2_H2 A3

// #define ST_PIN A2

#define MOTOR_0 0
#define MOTOR_1 1
#define MOTOR_BOTH 2

#define MOTOR_1_IRQ INT1_vect
#define MOTOR_1_PIN PINB
#define MOTOR_1_DIR PINB2

#define MOTOR_2_IRQ INT0_vect
#define MOTOR_2_PIN PINB
#define MOTOR_2_DIR PINB4

#define PULSE_PER_C 12 

EMotor motor[2];

#define RELAX_ANGLE -6.2 //自然平衡角度
#define MOTOR_ENABLE     //使能电机
const double PERIOD = 5000;


/***************注册标识位***************/
#define COMDONE 0x0001    //帧指令结束标识
#define MOVING 0x0002     //运动中标识
#define START 0x0008      //初始化完成标识

/***************MPU6050变量定义**********************/
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double  compAngleY, compAngleX, gyroXangle;
int16_t tempRaw;
uint32_t IMU_timer;
uint8_t i2cData[14];

/***************PID变量定义**********************/
typedef struct
{
  double P, I, D;
  double Setpoint, Output, Integral, last_error;
  uint32_t Timer;
} PID;

PID  PID_angle, PID_speed, PID_turn;

uint32_t FLAG;
char comdata[19], data_p; //串口缓存数据
float joy_x, joy_y, com_p, com_i, com_d;

void setup()
{
  MOTOR(0, 0);
  MOTOR(1, 0);
  /*********************初始化通信********************/
  Serial.begin(115200);  //蓝牙串口
  Wire.begin();           //I2C总线

 /*********************初始化电机驱动********************/
  pinMode(INT_1_PIN, INPUT_PULLUP);
  pinMode(DIR_1_PIN, INPUT_PULLUP);
  pinMode(INT_2_PIN, INPUT_PULLUP);
  pinMode(DIR_2_PIN, INPUT_PULLUP);

  pinMode(MOTOR_1_H1,OUTPUT);
  pinMode(MOTOR_1_H2,OUTPUT);
  pinMode(MOTOR_2_H1,OUTPUT);
  pinMode(MOTOR_2_H2,OUTPUT);

  EICRA = (1 << ISC11) | (1 << ISC01);
  EIMSK = (1 << INT0) | (1 << INT1);

  
  /*********************MPU6050初始化********************/
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)
    while (1);
  delay(100);

  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  tempRaw = (i2cData[6] << 8) | i2cData[7];

  double pitch = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  //gyroXangle = pitch;
  compAngleY = pitch;
  IMU_timer = micros();


  /*********************其他初始化********************/
  PID_angle.Setpoint = RELAX_ANGLE;
  PID_speed.Setpoint = 0;

  PID_angle.P = 12.5;
  PID_angle.I = 0.06;
  PID_angle.D = 0.0012;

  PID_turn.P = 1;
  //PID_turn.I = 0.01;
  PID_turn.D = 0.01;

  PID_revalue();

  IMU_fillter();
  
  FLAG |= START;
  
  protect();
}


void loop()
{
  //fixdelay();

  IMU_fillter();

  get_cmd();
  set_value();

  PID_revalue();
  
  protect();
  PID_angle_compute();
  PID_speed_compute();

  protect();
  PID_angle_compute();
  PID_turn_compute();

  protect();
  PID_angle_compute();
}

