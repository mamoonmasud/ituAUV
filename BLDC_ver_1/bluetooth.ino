void bluetooth()
{

  if (Serial2.available())
  {

    while (Serial2.available())
    {
      char inChar = (char)Serial2.read(); //read the input
      inputString += inChar;        //make a string of the characters coming on serial
    }
    Serial2.println(inputString);
    while (Serial2.available() > 0)
    {
      junk = Serial2.read() ;  // clear the serial buffer
    }
    if (inputString == "a")
    { //in case of 'a' turn the LED on
      digitalWrite(13, HIGH);
      Serial2.println("Initialization Successful");
    }
    else if (inputString == "b")

    { //incase of 'b' turn the LED off
      digitalWrite(13, LOW);
    }
    inputString = "";
  }

}
