void bluetooth_control()
{
  if (Serial2.available() >= 2)
  {
    if (Serial2.read() == '#') // Start of new control message
    {

      int command1 = Serial2.read(); // Commands
      if (command1 == 't') // tuning mode turn on
      {
        myPID.SetMode(AUTOMATIC);
        Serial2.println("Tuning mode for horizontal PID has been started");
        myPID.SetOutputLimits( 50 , 255);
        motion_control = 0;
        char output_param1 = Serial2.read();

        if (output_param1 == 'p')  // Set the Proportional Coefficent
        {
          Kp = Serial2.parseInt();
          Kp = Kp / 100;
          myPID.SetTunings(Kp, Ki, Kd);

          Serial2.print("The Proportional Coefficent is now:");
          Serial2.println(Kp);
        }
        else if (output_param1 == 'd')
        {
          Kd = Serial2.parseInt();
          Kd = Kd / 100;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial2.print("The Derivative Coefficent is now:");
          Serial2.println(Kd);
        }

        else if (output_param1 = 'i')           ///////// Tuning for Integral Coefficent
        {
          Ki = Serial2.parseInt();
          Ki = Ki / 100;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial2.print("The Integral Coefficent is now:");
          Serial2.println(Ki);
        }
        myPID.SetTunings(Kp, Ki, Kd); /// Function to Update the PID Tuning Parameters

      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      else if (command1 == 'q') // tuning mode turn on
      {
        vPID.SetMode(AUTOMATIC);
        Serial2.println("Tuning mode for vertical PID has been started");
        vPID.SetOutputLimits( 50 , 255);
        vert_control = 3;
        char output_param6 = Serial2.read();

        if (output_param6 == 'p')  // Set the Proportional Coefficent
        {
          Kpp = Serial2.parseInt();
          Kpp = Kpp / 100;
          vPID.SetTunings(Kpp, Kip, Kdp);

          Serial2.print("The Proportional Coefficent is now:");
          Serial2.println(Kpp);
        }

        else if (output_param6 == 'd')
        {
          Kdp = Serial2.parseInt();
          Kdp = Kdp / 100;
          vPID.SetTunings(Kpp, Kip, Kdp);
          Serial2.print("The Derivative Coefficent is now:");
          Serial2.println(Kdp);
        }

        else if (output_param6 = 'i')           ///////// Tuning for Integral Coefficent
        {
          Kip = Serial2.parseInt();
          Kip = Kip / 100;
          vPID.SetTunings(Kpp, Kip, Kdp);
          Serial2.print("The Integral Coefficent is now:");
          Serial2.println(Kip);
        }
        vPID.SetTunings(Kpp, Kip, Kdp); /// Function to Update the PID Tuning Parameters
      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      /////// Depth Setting using PID controller ///////
      else if (command1 == 'd')
      {
        vPID.SetMode(AUTOMATIC);
        Serial2.println("Depth Setting PID mode started");
        set_depth = Serial2.parseInt();
        set_depth = set_depth/100;
        vert_control = 4;

      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      else if (command1 == 'p') ///////////// Printing the PID tuning Parameters
      {
        Serial2.print("The PID parameters are: Kp : ");
        /// Setting the setpoint algorithm to be inserted here.
        //myPID.SetSetpoint(&readsetpoint);
        Serial2.print(Kp);
        Serial2.print("; Kd: ");
        Serial2.print(Kd);
        Serial2.print("; Ki:");
        Serial2.println(Ki);

        Serial2.print("The PID parameters of vertical control are: Kpp : ");
        Serial2.print(Kpp);
        Serial2.print("; Kdp: ");
        Serial2.print(Kdp);
        Serial2.print("; Kip:");
        Serial2.println(Kip);
      }
      //////////////////////////////////////////////////////////

      else if (command1 == 'y')         /// Printing the Yaw angle
      {
        Serial2.print("The current Yaw angle is: ");
        Serial2.println(yaw_1);

      }
      ////////////////////////////////////////////////////


      else if (command1 == 's') // stable mode turn on
      {
        myPID.SetMode(AUTOMATIC);
        motion_control = 1;
        Serial2.println("Stable mode has started");
        readsetpoint = Serial2.parseInt();


        /// Setting the setpoint algorithm to be inserted here.
        Setpoint = readsetpoint;

        Serial2.println("Setpoint is  now: ");
        Serial2.println(readsetpoint);
      }


      else if (command1 == 'c') // control mode turn on
      {
        myPID.SetMode(AUTOMATIC);

        myPID.SetOutputLimits( 0 , 150);

        char output_param2 = Serial2.read();

        if (output_param2 == 'f')  // move forward
        {
          motion_control = 2;
          fwd_yaw = yaw_1 ;
          fwdspeed = Serial2.parseInt();
          Serial2.println(" Forward Motion Started ");
        }

        else if (output_param2 == 'r')  // turn right by 90 degrees
        {
          fwd_yaw = yaw_1 + 90;

          motion_control = 3;
        }

        else if (output_param2 == 'l')  // turn left by 90 degrees
        {
          fwd_yaw = yaw_1 - 90;
          motion_control = 4;
        }

        else if (output_param2 == 'b') //  move backward
        {
          motion_control = 4;
        }

        else if (output_param2 == 'c') // move in a circle
        {

        }

      }


      else if (command1 == 'v') // vertical control mode
      {
        char output_param3 = Serial2.read();
        if (output_param3 == 'u')  // Move Upwards
        {
          vert_control = 1;
          vertical_speed = Serial2.parseInt();
          Serial2.print(" Upward Motion Started with speed: ");
          Serial2.println (vertical_speed);

        }


        else if (output_param3 == 'd')  // Move Downwards
        {
          vert_control = 2;
          vertical_speed = Serial2.parseInt();
          Serial2.print(" Downward Motion Started with speed: ");
          Serial2.println (vertical_speed);
        }
      }



      else if (command1 == 'h') // horizontal control mode
      {
        char output_param4 = Serial2.read();
        if (output_param4 == 'f')  // Move Forward
        {
          motion_control = 10;
          horizontal_speed = Serial2.parseInt();
          Serial2.print(" Forward Motion Started with speed: ");
          Serial2.println (horizontal_speed);
        }

        else if (output_param4 == 'b')  // Move Backwards
        {
          motion_control = 11;
          horizontal_speed = Serial2.parseInt();
          Serial2.print(" Backward Motion Started with speed: ");
          Serial2.println (horizontal_speed);
        }
        //cali_hori_speed = ((-1 * horizontal_speed) + 256);
      }

      else if (command1 == 'r') // rotational control mode
      {
        char output_param5 = Serial2.read();
        if (output_param5 == 'c')  // Move Clockwise
        {
          motion_control = 14;
          rotational_speed = Serial2.parseInt();
          Serial2.print(" Clockwise Motion Started with speed: ");
          Serial2.println (rotational_speed);

        }

        else if (output_param5 == 'a')  // Move Anti-Clockwise
        {
          motion_control = 15;
          rotational_speed = Serial2.parseInt();
          Serial2.print(" Anticlockwise Motion Started with speed: ");
          Serial2.println (rotational_speed);

        }

        else if (output_param5 == 'r') // move in a circle in right direction.

        {
          motion_control = 16;
          circular_speed = Serial2.parseInt();
          Serial2.println(" Clockwise Circular Motion Started");
        }

        else if (output_param5 == 'l') // move in a circle in right direction.

        {
          motion_control = 17;
          circular_speed = Serial2.parseInt();
          Serial2.println(" Anti-clockwise Circular Motion Started");
        }
      }

      //////
      /// Angle Setting using PID control ///


      else if (command1 == 'b') // angle setting control
      {
        myPID.SetMode(AUTOMATIC);

        Serial2.println("Angle Setting PID Started");

        //myPID.SetOutputLimits( 50 , 255);
        motion_control  = 18;
        myPID.SetTunings(Kp, Ki, Kd);

        rot_angle       = Serial2.parseInt();
        Serial2.print(" The AUV is now directed at: ");
        Serial2.println (rot_angle);

        if (rot_angle > 180 )
        {
          rot_angle = rot_angle - 360;
        }

      }


      ///////////////////////////

      else if (command1 == 'a' )// STOP Everything
      {

        Serial2.println(" Motion of AUV has ceased. ");
        motion_control = 9;  // STOP
        vert_control = 0;

        //myPID.SetMode(MANUAL);
        Output = 0;
      }

      else if (command1 == 'g') // Recalibrate the motors
      {
        Serial2.println("Motor being calibrated");
        
        servo_a.writeMicroseconds(1500); // send "stop" signal to ESC.
        servo_f.writeMicroseconds(1500);
        servo_r.writeMicroseconds(1500);
        servo_l.writeMicroseconds(1500);

        delay (1000);

        servo_a.writeMicroseconds(2000); // send "Maximum Positive" signal to ESC.
        servo_f.writeMicroseconds(2000);
        servo_r.writeMicroseconds(1500);
        servo_l.writeMicroseconds(1500);

        delay (1000);

        servo_a.writeMicroseconds(1500); // send "stop" signal to ESC.
        servo_f.writeMicroseconds(1500);
        servo_r.writeMicroseconds(2000);
        servo_l.writeMicroseconds(2000);

        delay (1000);

        servo_a.writeMicroseconds(1200); // send "stop" signal to ESC.
        servo_f.writeMicroseconds(1200);
        servo_r.writeMicroseconds(1500);
        servo_l.writeMicroseconds(1500);

        delay (1000);

        servo_a.writeMicroseconds(1500); // send "stop" signal to ESC.
        servo_f.writeMicroseconds(1500);
        servo_r.writeMicroseconds(1200);
        servo_l.writeMicroseconds(1200);

        delay (1000);

        servo_a.writeMicroseconds(1500); // send "stop" signal to ESC.
        servo_f.writeMicroseconds(1500);
        servo_r.writeMicroseconds(1500);
        servo_l.writeMicroseconds(1500);

        delay (1000);

        Serial.print(" Motor Calibration completed.");
        motion_control = 5;

      }
      else if (command1 == 'e') // Reset the tuning parameters
      {
        Kp = 0, Ki = 0, Kd = 0;
        Kpp = 0, Kip = 0, Kdp = 0;

      }
    }
  }
}
