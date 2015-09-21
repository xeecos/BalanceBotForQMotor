double gyroZrate = 0;
double roll = 0;
double pitch = 0;
//int32_t i,add;
void IMU_fillter(void)
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - IMU_timer) / 1000000;
  IMU_timer = micros();

  pitch = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  roll = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double gyroXrate = gyroX / 131.0;
  double gyroYrate = gyroY / 131.0;
  gyroZrate += ( (double)gyroZ +43.5 ) / 131.0 * dt;
 
  //gyroXangle += gyroXrate * dt;
  /*add += gyroZ;
  i++;
  if(i==500)
  {
    Serial.println((double)add / i);
    Serial.println(gyroZrate);
    i=0;
    add = 0;
  }*/
  compAngleY = 0.98 * (compAngleY + gyroXrate * dt) + 0.02 * pitch;
  compAngleX = 0.98 * (compAngleX + gyroYrate * dt) + 0.02 * roll;
  //Serial.println(compAngleX);
}

void protect()
{
  #if defined MOTOR_ENABLE
  if (FLAG & START)
  {
    //roll = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    if( compAngleY > ( 20 + RELAX_ANGLE ) ||  compAngleY < ( -20 + RELAX_ANGLE ) || compAngleX > 25 || compAngleX < -25 )
    //if( pitch > 80 || pitch < -80 || roll > 75 || roll < -75 )
    {
      MOTOR(0, 0);
      MOTOR(1, 0);
      while( !( compAngleY < ( 0.15 + RELAX_ANGLE ) && compAngleY > ( -0.15 + RELAX_ANGLE ) && compAngleX < 10 && compAngleX > -10 ) )
      {
        IMU_fillter();
        roll = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
        /*Serial.print("pitch:");
        Serial.println(pitch);
        Serial.print("roll");
        Serial.println(roll);
        Serial.println();*/
        MOTOR(0, 0);
        MOTOR(1, 0);
      }
      PID_angle.Integral = 0;
      PID_speed.Integral = 0;
      gyroZrate = 0;
      PID_angle.Setpoint = RELAX_ANGLE;
      PID_speed.Setpoint = 0;
      PID_turn.Setpoint = 0;
    }
  }
  #endif
}

