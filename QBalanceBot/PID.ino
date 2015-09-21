void PID_revalue(void)
{
  if (FLAG & MOVING)
  {
    PID_speed.P = 0.025;
    PID_speed.I = 0.0012;
  }
  else
  {
    PID_speed.P = 0.022;
    PID_speed.I = 0.001;
    //PID_speed.P = 0.0;
    //PID_speed.I = 0.0;
  }
}

void PID_angle_compute(void)
{
  double dt = micros() - PID_angle.Timer ;
  if ( dt > PERIOD )
  {
    double error = compAngleY - PID_angle.Setpoint;
    PID_angle.Integral += error * dt * 0.001;
    constrain(PID_angle.Integral, -255, 255);

    PID_angle.Output = PID_angle.P * error + PID_angle.I * PID_angle.Integral - PID_angle.D * gyroX;

    double pwm_left = PID_angle.Output + PID_turn.Output ;
    double pwm_right = PID_angle.Output - PID_turn.Output ;

    constrain(pwm_left, -120, 120);
    constrain(pwm_right, -120, 120);

#ifdef MOTOR_ENABLE
    MOTOR(0, pwm_left);
    MOTOR(1, pwm_right);
#endif

    PID_angle.Timer = micros();
  }
}

void PID_speed_compute(void)
{
  double dt = micros() - PID_speed.Timer ;

  if (dt > ( PERIOD * 10 ) )
  {
    updateSpeed(&motor[0]);
    updateSpeed(&motor[1]);
    //Serial.println("*******");
    //Serial.println(motor[0].speed);
    //Serial.println(motor[1].speed);
    double speed_now = ( (double)motor[1].speed / 357.3 / (dt / 1000000) * 60 ) - ( (double)motor[0].speed / 357.3 / (dt / 1000000) * 60 );
    //Serial.println(speed_now);
    double error = speed_now - PID_speed.Setpoint ;
    PID_speed.Integral += error;

    PID_speed.Output = PID_speed.P * speed_now + PID_speed.I * PID_speed.Integral ;
    constrain(PID_speed.Output , -35, 35);
    PID_angle.Setpoint =  RELAX_ANGLE +  PID_speed.Output ;

    PID_speed.Timer = micros();
    //Serial.println(error);
    /*Serial.println("***************");
    Serial.println(PID_speed.P);
    Serial.println(PID_speed.I);
    Serial.println(PID_speed.D);*/
  }
}


void PID_turn_compute(void)
{
  double dt = micros() - PID_turn.Timer ;
  if (dt > ( PERIOD * 10 ) )
  {
    double error = PID_turn.Setpoint - gyroZrate;
    PID_turn.Integral += error;
    double delta = error - PID_turn.last_error;
    PID_turn.last_error = error;

    PID_turn.Output = PID_turn.P * error + + PID_turn.I * PID_speed.Integral + PID_turn.D * delta ;

    PID_turn.Timer = micros();
  }
}

