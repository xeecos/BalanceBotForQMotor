void setMotor1Pwm(int pwm)
{
  pwm = constrain(pwm,-255,255);
  if(pwm>0)
  {
    digitalWrite(MOTOR_1_H1, LOW);
    digitalWrite(MOTOR_1_H2, HIGH);
    analogWrite(MOTOR_1_PWM, abs(pwm));
  }
  else
  {
    digitalWrite(MOTOR_1_H1, HIGH);
    digitalWrite(MOTOR_1_H2, LOW);
    analogWrite(MOTOR_1_PWM, abs(pwm));
  }
}

void setMotor2Pwm(int pwm)
{
  pwm = constrain(pwm,-255,255);
  if(pwm>0)
  {
    digitalWrite(MOTOR_2_H1, LOW);
    digitalWrite(MOTOR_2_H2, HIGH);
    analogWrite(MOTOR_2_PWM, abs(pwm));
  }
  else
  {
    digitalWrite(MOTOR_2_H1, HIGH);
    digitalWrite(MOTOR_2_H2, LOW);
    analogWrite(MOTOR_2_PWM, abs(pwm));
  }
}

void updateSpeed(EMotor * m)
{
  m->speed = m->pos - m->posSpeed;
  m->posSpeed = m->pos;
}

#define FIXDELAY 10*1000 // 10ms loop delay
long time,deltaTime;
void fixdelay()
{
  deltaTime = micros()-time;
  delayMicroseconds(FIXDELAY-deltaTime);
  time = micros();
}

ISR(MOTOR_1_IRQ)
{
  if(MOTOR_1_PIN & (1<<MOTOR_1_DIR))
  {
    motor[0].pos--;
  }
  else
  {
    motor[0].pos++;
  }
}

ISR(MOTOR_2_IRQ)
{
  if(MOTOR_2_PIN & (1<<MOTOR_2_DIR))
  {
    motor[1].pos--;
  }
  else
  {
    motor[1].pos++;
  }
}


