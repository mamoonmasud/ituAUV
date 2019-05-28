/* This file is part of the Razor AHRS Firmware */

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  //////// For GY-85 Tilt Compensation //////////
  float x_new = magnetom[1];
  float y_new = -magnetom[0];
  float z_new = magnetom[2];
  ///////////////////////////////////////////////

  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);

  /*
      Original Part of the Code. It's not useful for the GY-85 since the Orientation of HMC5883L is different in GY-85 as compared to the SEN-10724
    // Tilt compensated magnetic field X
    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y
    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  */

  //  Tilt Compensation after catering for the differnce in the magnetometer's orientation

  // Tilt compensated magnetic field X
  mag_x = x_new * cos_pitch + y_new * sin_roll * sin_pitch + z_new * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = y_new * cos_roll - z_new * sin_roll;



  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
}
