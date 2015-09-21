void MOTOR(char TYPE, int SPEED)
{
  if (!TYPE)//type==0,取反
  {
    setMotor1Pwm(SPEED);
  }
  else
  {
    setMotor2Pwm(-SPEED);
  }
}

