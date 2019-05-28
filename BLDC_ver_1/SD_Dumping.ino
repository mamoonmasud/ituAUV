void sd_data()

{
  if (millis() - sdTime > SDWriteinterval)
  {

    //    unsigned long lat1 = latitude;
    //    unsigned long lon1 = longitude;
    //    String latbuf1;
    //    latbuf1 += (String(lat1, 6));
    //    String lonbuf1;
    //    lonbuf1 += (String(lon1, 6));


    String dataString = "{ ";
    //dataString = millis();
    dataString += "\"Flow total\": ";
    dataString += String (totalMilliLitres) ;
    dataString += ", \"Current flow\": ";
    dataString += String (flowMilliLitres);

    dataString += ", \"pH\": ";
    dataString += String(pHValue, 2);

    dataString += ", \"turbidity\": ";
    dataString += String ( turbidity_cal);
    dataString += ", \"total dissolved solid\": ";
    dataString += String (tdsValue);


    dataString += ", \"Conductivity\": ";
    dataString += String (cond);
    dataString += ", \"temperature\": ";
    dataString += String (temp_cls);
    dataString += ", \"longitude\": ";
    dataString += String (lonbuf);
    dataString += ", \"latitude\": ";
    dataString += String (latbuf);

    dataString += ", \"P_Temperature\": ";
    dataString += String (p_temp);
    dataString += ", \"P_Pressure\": ";
    dataString += String (p_pressure);
    dataString += ", \"P_Depth\": ";
    dataString += String (p_depth);
    dataString += ", \"P_Altitude\": ";
    dataString += String (p_altitude);

    dataString += " } ,";

    //    dataString += ";";
    //    dataString += lonbuf1;
    //    dataString += ";";
    //    dataString += latbuf1 ;
    //    /// dataString += ";";
    //    dataString += ";flow: ";
    //    dataString += String (totalMilliLitres);
    //    dataString += ";";
    //
    //    dataString += String (flowMilliLitres);
    //    dataString += ";";
    //
    //    dataString += String (pHValue, 2);
    //
    //    dataString += ";";
    //    dataString += String (turbidity_cal);
    //    dataString += ";";
    //    dataString += String (tdsValue);
    //    dataString += ";";
    //    dataString += String (cond);
    //    dataString += ";";
    //    dataString += String (temp_cls);


    DataFile = SD.open (fileName, FILE_WRITE);

    if (DataFile)
    {
      DataFile.println(dataString);
      DataFile.close();
      //Serial.println(dataString);
      printTime = millis();
    }

    else
    {
      Serial.println("error opening datalog.txt");

    }

  }
}

