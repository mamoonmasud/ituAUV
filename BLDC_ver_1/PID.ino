///////// Main Speed controlling Code --- Switch Case to be added to this code to make it better instead of if else statements //////////

/////////////////////////// Code for Controling the speed of motors using a PID Controller /////////////////////////
int pitch_adj = pitch_1;


void PID_control()
{

  if  (motion_control == 0)               // Tuning Mode for the PID Controller, changing the values for the tuning of PID Controllers
  {
    myPID.SetOutputLimits( 0 , 255);
    if (yaw_1 > 0)
    {
      yaw_adj = -1 * yaw_1;
    }
    else
    {
      yaw_adj = yaw_1;
    }
    Input = yaw_adj ;
    myPID.Compute() ;

    cali_vout = ((1.804 * Output) + 1540);
    cali_vout2 = ((-0.9804 * Output) + 1450);

    if ( yaw_1 > (-1 * window) && yaw_1 <= 0 || yaw_1 < window && yaw_1 > 0 ) //////// The condition for stability - Motors Turn off
    {
      Serial.println("This code is running");
      servo_r.writeMicroseconds(1500);
      servo_l.writeMicroseconds(1500);
    }

    else if ( yaw_1 + window < 0 ) ///// The condition for clockwise motion - angle is negative.
    {
      servo_r.writeMicroseconds(cali_vout2);  // (Right motor - backward direction)
      servo_l.writeMicroseconds(cali_vout);  // (Left motor - forward direction)
    }

    else if (yaw_1 - window > 0)   ///// The condition for anti-clockwise motion - angle is positive.
    {
      servo_r.writeMicroseconds(cali_vout);  // (Right motor - forward direction)
      servo_l.writeMicroseconds(cali_vout2);  // (Left motor - backward direction)
    }

    Serial.println(cali_vout);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  else if (motion_control == 1)     // Stable mode for defined axis along Z-axis
  {

    myPID.SetOutputLimits (0, 150); // If this doesn't work, then just make a new formula for cali_vout and cali_vout2

    //////////////////

    base_speed = 1700;

    // Set point is readsetpoint for this code-block

    angle_adj = angle_cor( yaw_1 , readsetpoint);

    if (angle_adj < 0)
    {
      yaw_adj = angle_adj;
    }
    else
    {
      yaw_adj  = -1 * angle_adj;
    }
    Input = yaw_adj;
    myPID.Compute();

    //    cali_vout = ((1.804 * Output_p) + 1540);
    //    cali_vout2 = ((-0.9804 * Output_p) + 1450);

    if (( yaw_1  > ((-1 * window) + readsetpoint) ) && yaw_1  <= readsetpoint || yaw_1 > (window + readsetpoint) && yaw_1  < readsetpoint )
    {
      Serial.println("This code is running");
      servo_r.writeMicroseconds(base_speed);
      servo_l.writeMicroseconds(base_speed);
    }

    ///// SPEED Needs to be adjusted.
    // int leftMotorSpeed = 1500 - iniMotorPower - PIDvalue;

    else if ( angle_adj  < 0)     // The AUV has moved slightly towards the left and needs to be moved slightly to the right (clockwise motion)
    {
      servo_r.writeMicroseconds(base_speed - Output );
      servo_l.writeMicroseconds(base_speed + Output);
    }

    else if (angle_adj  >= 0)      // The AUV has moved slightly towards the right and needs to be moved slightly to the left (anti-clockwise motion)
    {
      servo_r.writeMicroseconds(base_speed + Output);
      servo_l.writeMicroseconds(base_speed - Output);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  else if   (motion_control == 2)               // Forward Mode
  {
    myPID.SetOutputLimits( 0 , 150);

    // fwd_yaw is the variable storing the current angle, which is the setpoint.
    // fwdspeed contains the base speed.

    angle_adj = angle_cor( yaw_1 , fwd_yaw);

    if (angle_adj < 0)
    {
      yaw_adj = angle_adj;
    }
    else
    {
      yaw_adj  = -1 * angle_adj;
    }
    Input = yaw_adj;
    myPID.Compute();

    if ( angle_adj  > (-1 * window) && angle_adj  <= 0 || angle_adj > (window) && angle_adj  < 0 )
    {
      Serial.println("The AUV is moving forward with a constant speed");
      servo_r.writeMicroseconds(fwdspeed);
      servo_l.writeMicroseconds(fwdspeed);
    }
    else if ( angle_adj < 0 )   // The AUV has moved slightly towards the left and needs to be moved slightly to the right (clockwise motion)
    {
      servo_r.writeMicroseconds(fwdspeed - Output );
      servo_l.writeMicroseconds(fwdspeed + Output);
    }
    else if (angle_adj > 0)  // The AUV has moved slightly towards the right and needs to be moved slightly to the left (anti-clockwise motion)
    {
      servo_r.writeMicroseconds(fwdspeed + Output);
      servo_l.writeMicroseconds(fwdspeed - Output);
    }
  }

  ///////////////////////////////////////////////////////////////////////___// Turn Right by 90* \\___\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

  else if   (motion_control == 3)
  {
    myPID.SetOutputLimits( 0 , 250);

    // fwd_yaw is the variable storing the current angle with a 90 degree offset, which is the required angle for rotating right 90 degrees.
    // basespeed is zero

    angle_adj = angle_cor( yaw_1 , fwd_yaw);

    if (angle_adj < 0)
    {
      yaw_adj = angle_adj;
    }
    else
    {
      yaw_adj  = -1 * angle_adj;
    }
    Input = yaw_adj;
    myPID.Compute();

    cali_vout = ((1.804 * Output) + 1540);
    cali_vout2 = ((-0.9804 * Output) + 1450);

    if ( angle_adj  > (-1 * window) && angle_adj  <= 0 || angle_adj > (window) && angle_adj  < 0 )
    {
      Serial.println("The AUV has turned right");
      servo_r.writeMicroseconds(1500);
      servo_l.writeMicroseconds(1500);
    }
    else if ( angle_adj < 0 )   // The AUV needs to be moved clockwise.
    {
      servo_r.writeMicroseconds(cali_vout2);
      servo_l.writeMicroseconds(cali_vout );
    }
    else if (angle_adj > 0)  // The AUV has moved slightly towards the right and needs to be moved slightly to the left (anti-clockwise motion)
    {
      servo_r.writeMicroseconds(cali_vout );
      servo_l.writeMicroseconds(cali_vout2);
    }
  }

  ///////////////////////////////////////////////////////////////////////___// Turn Left by 90* \\___\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

  else if   (motion_control == 4)
  {
    myPID.SetOutputLimits( 0 , 250);

    // fwd_yaw is the variable storing the current angle with a 90 degree offset, which is the required angle for rotating right 90 degrees.
    // basespeed is zero

    angle_adj = angle_cor( yaw_1 , fwd_yaw);

    if (angle_adj < 0)
    {
      yaw_adj = angle_adj;
    }
    else
    {
      yaw_adj  = -1 * angle_adj;
    }
    Input = yaw_adj;
    myPID.Compute();

    cali_vout = ((1.804 * Output) + 1540);
    cali_vout2 = ((-0.9804 * Output) + 1450);

    if ( angle_adj  > (-1 * window) && angle_adj  <= 0 || angle_adj > (window) && angle_adj  < 0 )
    {
      Serial.println("The AUV has turned right");
      servo_r.writeMicroseconds(1500);
      servo_l.writeMicroseconds(1500);
    }
    else if ( angle_adj < 0 )   // The AUV needs to be moved clockwise.
    {
      servo_r.writeMicroseconds(cali_vout2);
      servo_l.writeMicroseconds(cali_vout );
    }
    else if (angle_adj > 0)  // The AUV has moved slightly towards the right and needs to be moved slightly to the left (anti-clockwise motion)
    {
      servo_r.writeMicroseconds(cali_vout );
      servo_l.writeMicroseconds(cali_vout2);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  else if   (motion_control == 10)               // Forward Motion
  {
    //// horizontal_speed contains speed of AUV. (1540 min, 2000 max)
    servo_r.writeMicroseconds(horizontal_speed );
    servo_l.writeMicroseconds(horizontal_speed);
  }

  else if   (motion_control == 11)               // Backward Motion
  {
    servo_r.writeMicroseconds(horizontal_speed );
    servo_l.writeMicroseconds(horizontal_speed);
  }

  else if (motion_control == 14)                //Clockwise Motion
  {
    cali_vout   = ((1.804   * rotational_speed) + 1540);
    cali_vout2  = ((-0.9804 * rotational_speed) + 1450);

    servo_r.writeMicroseconds(cali_vout2 );
    servo_l.writeMicroseconds(cali_vout);
  }

  else if (motion_control == 15)                //Anti Clockwise Motion
  {
    cali_vout   = ((1.804   * rotational_speed) + 1540);
    cali_vout2  = ((-0.9804 * rotational_speed) + 1450);

    servo_r.writeMicroseconds(cali_vout  );
    servo_l.writeMicroseconds(cali_vout2 );
  }

  else if (motion_control == 16)                //Circular Motion in Right Direction
  {
    //circular_speed has the difference of speed.
    servo_r.writeMicroseconds(1650 - circular_speed);
    servo_l.writeMicroseconds(1650 + circular_speed);
  }

  else if (motion_control == 17)                //Circular motion in left Direction
  {
    servo_r.writeMicroseconds(1650 + circular_speed);
    servo_l.writeMicroseconds(1650 - circular_speed);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  else if (motion_control == 18)                    //////// Angle Setting PID mode
  {
    myPID.SetOutputLimits( 0 , 150);

    // rot_angle is the variable storing the required angle, which is the setpoint.


    angle_adj = angle_cor( yaw_1 , rot_angle);

    if (angle_adj < 0)
    {
      yaw_adj = angle_adj;
    }
    else
    {
      yaw_adj  = -1 * angle_adj;
    }
    Input = yaw_adj;
    myPID.Compute();

    cali_vout = ((1.804 * Output) + 1540);
    cali_vout2 = ((-0.9804 * Output) + 1450);

    if ( angle_adj  > (-1 * window) && angle_adj  <= 0 || angle_adj > (window) && angle_adj  < 0 )
    {
      Serial.println("The AUV is set at the desired angle");
      servo_r.writeMicroseconds(1500);
      servo_l.writeMicroseconds(1500);
    }
    else if ( angle_adj < 0 )   // The AUV needs to be moved to the right (clockwise motion)
    {
      servo_r.writeMicroseconds(cali_vout2);
      servo_l.writeMicroseconds(cali_vout);
    }
    else if (angle_adj > 0)  // The AUV needs to be moved slightly to the left (anti-clockwise motion)
    {
      servo_r.writeMicroseconds(cali_vout);
      servo_l.writeMicroseconds(cali_vout2);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////


  }


  else if   (motion_control == 9)               // STOP Mode
  {

    servo_a.writeMicroseconds(1500); // send "stop" signal to ESC.
    servo_f.writeMicroseconds(1500);
    servo_r.writeMicroseconds(1500);
    servo_l.writeMicroseconds(1500);
    //myPID.SetOutputLimits (0,0);
  }

  else if (motion_control == 5)

  {
    

  }


  //////////////////////////////////////////////////////////////////
  ///// Commands for Vertical Control - Without PID Controller /////
  //////////////////////////////////////////////////////////////////

  if   (vert_control == 1)                  // Upward Motion
  {
    servo_a.writeMicroseconds(vertical_speed);
    servo_f.writeMicroseconds(vertical_speed);
  }


  else  if   (vert_control == 2)               // Downward Motion
  {
    servo_a.writeMicroseconds(vertical_speed);
    servo_f.writeMicroseconds(vertical_speed);
  }

  else  if   (vert_control == 0)               // STOP Motion
  {
    servo_a.writeMicroseconds(1500);
    servo_f.writeMicroseconds(1500);
  }

  /////////////////////////////////
  ////////////////////////////////////////////// VERTICAL CONTROL - PID Tuning MODE ////////////////////////////////////////////////
  else if    (vert_control == 3)              // Tuning Mode
  {
    vPID.SetOutputLimits( 0 , 255);
    
    if (p_depth > 0)
    {
      depth_adj = -1 * p_depth;
    }
    else
    {
      depth_adj = p_depth;
    }
    Input_p = depth_adj ;
    vPID.Compute() ;

    cali_vout = ((1.804 * Output_p) + 1540);
    cali_vout2 = ((-0.9804 * Output_p) + 1450);

    if ( p_depth > (-1 * v_win) && p_depth < (v_win)) //////// The condition for stability - Motors Turn off
    {
      Serial.println("Vertical Motors stop");
      servo_a.writeMicroseconds(1500);
      servo_f.writeMicroseconds(1500);
    }

    else if ( p_depth + v_win < 0 ) ///// The depth is negative (in air), i.e. vehicle needs to move downwards
    {
      servo_a.writeMicroseconds(cali_vout);
      servo_f.writeMicroseconds(cali_vout);
    }

    else if ( p_depth - v_win > 0)   ///// The depth is positive, i.e. vehicle needs to move upwards
    {
      servo_a.writeMicroseconds(cali_vout2);
      servo_f.writeMicroseconds(cali_vout2);
    }
    Serial.println(p_depth);
    Serial.print("cali_vout:");
    Serial.print(cali_vout);
    Serial.print("cali_vout2:");
    Serial.println(cali_vout2);
  }


  ////////////////////////////////////////////////////////////////////////
  else if (vert_control == 4)       /// Depth setting PID Mode.
  {
    vPID.SetOutputLimits(0, 255);
    crc_depth = depth_cor(p_depth, set_depth);

    if (crc_depth  > 0)
    {
      depth_adj = -1 * crc_depth;
    }
    else
    {
      depth_adj = crc_depth ;
    }

    Input_p = depth_adj ;
    vPID.Compute() ;

    cali_vout = ((1.804 * Output_p) + 1540);
    cali_vout2 = ((-0.9804 * Output_p) + 1450);

    if ( crc_depth > (-1 * v_win) && crc_depth <= 0 || crc_depth < v_win && crc_depth > 0 ) //////// The condition for stability - Motors Turn off
    {
      Serial.println("Vertical Motors stop");
      servo_a.writeMicroseconds(1500);
      servo_f.writeMicroseconds(1500);
    }

    else if ( crc_depth + v_win < 0 ) ///// The depth is negative (in air), i.e. vehicle needs to move downwards
    {
      servo_a.writeMicroseconds(cali_vout);
      servo_f.writeMicroseconds(cali_vout);
    }

    else if (crc_depth - v_win > 0)   ///// The depth is positive, i.e. vehicle needs to move upwards
    {
      servo_a.writeMicroseconds(cali_vout2);
      servo_f.writeMicroseconds(cali_vout2);
    }
    Serial.println(p_depth);
  }
}

/////////////////////////////////////////////////////////////////////////
double angle_cor(double agle, double spoint)
{
  double correction = 0;
  correction = agle - spoint;
  if (correction > 180)
  {
    correction = correction - 360;
  }
  else if (correction < -179)
  {
    correction  = correction + 360;
  }
  return correction;
}

double depth_cor (double dpt, double sdpt)
{
  double cor_depth = 0;
  cor_depth = dpt - sdpt;
  return cor_depth;
}

