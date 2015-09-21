void get_cmd(void)
{
  while (Serial.available() > 0)
  {
    //Serial.println("ok");
    char nullByte = char(Serial.read());
    if (nullByte == ';')
    {
      comdata[data_p] = nullByte;
      data_p = 0;
      FLAG |= COMDONE;
    }
    else
    {
      comdata[data_p] = nullByte ;
      data_p++ ;
    }
  }
}

void set_value(void)
{
  if (FLAG & COMDONE)
  {
    if(comdata[0] == 'C')
    {
      //Serial.println("COMDONE");
      strtok(comdata, ",");
      joy_x = atof(strtok(NULL, ","));
      joy_y = atof(strtok(NULL, ";"));
      switch (comdata[1])
      {
        case 'J':
          FLAG |= MOVING;
          PID_speed.Setpoint = joy_y * 700;
          if (joy_x > 0.2 || joy_x < -0.2)
          {
            /*if (PID_speed.Setpoint > 0)
              PID_turn.Output  = joy_x * 50;
            else*/
            if (joy_x > 0.9 || joy_x < -0.9)
            {
              PID_turn.Setpoint  += -joy_x * 35;
            }
            else
            {
              PID_turn.Setpoint  += -joy_x * 20;
            }
          }
          //else
            //PID_turn.Setpoint  = 0;
          break;
        case 'S':
          PID_speed.Setpoint = 0;
          PID_turn.Output  = 0;
          FLAG &= ~MOVING;
          break;

      }
    }
    else if(comdata[0] == 'P')
    {
      strtok(comdata, ",");
      com_p = atof( strtok(NULL, ",") );
      com_i = atof( strtok(NULL, ";") );
      com_d = atof( strtok(NULL, ";") );
      switch (comdata[1])
      {
        case 'A':
          PID_angle.P = (double)com_p;
          PID_angle.I  = (double)com_i;
          PID_angle.D  = (double)com_d;
          FLAG |= MOVING;
          PID_angle.Integral = 0;
          PID_speed.Integral = 0;
          Serial.println("AP****************ID");
          break;
        case 'S':
          PID_speed.P = (double)com_p;
          PID_speed.I  = (double)com_i;
          PID_speed.D  = (double)com_d;
          FLAG |= MOVING;
          PID_speed.Integral = 0;
          Serial.println("SP****************ID");
          break;
      }
      com_p = 0;
      com_i = 0;
      com_d = 0;
    }
    FLAG &= ~COMDONE;
  }
}
