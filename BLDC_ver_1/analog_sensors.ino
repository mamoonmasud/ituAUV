

//// The function that calculates the approximate temperature in Celsius using the Thermistor attached to the AUV.
//// The thermistor pin is defined in the main function as A6, which is the default one. To change it, kindly change it in the main function.
//// The formula used to calculate the temperature in fahrenheit from raw ADC values has been approximated after calibrating the thermistor. A Power function has been found to give best approximation.

void temperature()
{
  temp_analog = analogRead(THERMISTORPIN);

  RunAvgBuftemp[NextRunAvgtemp++] = temp_analog;
  if (NextRunAvgtemp >= RunAvgTempCnt)
  {
    NextRunAvgtemp = 0;
  }

  float RunAvgtemp = 0;

  for (int i = 0; i < RunAvgTempCnt; ++i)
  {
    RunAvgtemp += RunAvgBuftemp[i];
  }
  RunAvgtemp /= RunAvgTempCnt;

  //  temp_fht   = 0.26 * ( pow(RunAvgtemp , 0.89));
  temp_fht   = 0.1693 * ( pow(RunAvgtemp , 1.004));

  temp_cls   = (temp_fht - 32) * 0.5556;
}


//////// Function to calculate turbidiyu using the turbidity sensor attached to the AUV.
//////// Turbidity pin is defined in the main function as Analog Pin A2 of Arduino Mega with which the Turbidty sensor's Analog Output Pin has been attached.

void turbidity()

{
  int rawTurbidity = 0;
  float tur_voltage = 0.0;

  rawTurbidity = analogRead(TURBIDITYPIN);// read the input on analog pin 0:
  tur_voltage = rawTurbidity * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  turbidity_cal = (-1120.4 * (pow(tur_voltage, 2)) + (5742.3 * tur_voltage) - 4352.9); // relationship of NTU with the voltage (0-5V) is -1120.4x^2 + 5742.3x - 4352.9

}


void TDS()
{
  float a = analogRead(TdsSensorPin);
  //Serial.println(a);
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffertds[analogBufferIndextds] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndextds++;
    if (analogBufferIndextds == SCOUNT)
      analogBufferIndextds = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndextds = 0; copyIndextds < SCOUNT; copyIndextds++)
      analogBufferTemptds[copyIndextds] = analogBuffertds[copyIndextds];
    averageVoltagetds = getMedianNum(analogBufferTemptds, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temp_cls - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltagetds / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    cond = (1.2686 * tdsValue) + 131.3929;
  }
}


int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}



