void p_sens()
{
  p_sensor.read();
  p_pressure  =   p_sensor.pressure();
  p_temp      =   p_sensor.temperature();
  p_depth     =   p_sensor.depth();
  p_altitude  =   p_sensor.altitude();
}


