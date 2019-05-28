
void gp()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    //while(true);
  }
}


void displayInfo()
{
  // Serial.print(F("Location: "));
  if (gps.location.isValid())
  {

    latitude = (gps.location.lat());
    longitude = (gps.location.lng());


    latbuf += (String(latitude, 6));
    //    Serial.println(latbuf);


    lonbuf += (String(longitude, 6));
    //    Serial.println(lonbuf);
    delay(1000);
  }
  
  else
  {
    Serial.print(F("INVALID"));
  }

  //  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    //    Serial.print(gps.date.month());
    //    Serial.print(F("/"));
    //    Serial.print(gps.date.day());
    //    Serial.print(F("/"));
    //    Serial.print(gps.date.year());
  }
  else
  {
    //    Serial.print(F("INVALID"));
  }

  //  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      //    Serial.print(F("0"));
      //    Serial.print(gps.time.hour());
      //    Serial.print(F(":"));
      if (gps.time.minute() < 10)
        //    Serial.print(F("0"));
        //    Serial.print(gps.time.minute());
        //    Serial.print(F(":"));
        if (gps.time.second() < 10)
          //    Serial.print(F("0"));
          //    Serial.print(gps.time.second());
          //    Serial.print(F("."));
          if (gps.time.centisecond() < 10) ;
    //    Serial.print(F("0"));
    //    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  //  Serial.println();
}
