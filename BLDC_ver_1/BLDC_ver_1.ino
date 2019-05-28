/*
  ///////////////////////////////////////////
  /////// Arduino Mega Pin Connections //////////

  /////////////////////////
      Servo Pins for the ESC's Connections.
  Back Motor  - Pin 4
  Front Motor - Pin 5

  Right Motor - Pin 6
  Left  Motor - Pin 7
  /////////////////////////
     GY-85
  SDA - Pin 20
  SCL - Pin 21

  /////////////////////////
      GPS Module
  Serial 1
  RX Pin -> Mega Pin 18(TX1)
  TX Pin -> Mega Pin 19(RX1)

  /////////////////////////
      Enviornmental Sensors

  pH Sensor - Analog Pin A0
  TDS Meter - Analog Pin A1
  Turbidity Sensor - Analog Pin A2
  Temperature Sensor - Analog Pin A3

  /////////////////////////
      Bluetooth Module
  Connected To Serial 2
  RX -> Mega Pin 16 (TX2)
  TX -> Mega Pin 17 (RX2)

  ////////////////////////
    SD Card Module
    ** MOSI - pin 51
    ** MISO - pin 50
    ** CLK - pin 52
    ** CS - pin 53

  ////////////////////////
    Water Pressure Sensor
    ** Yellow - SDA Pin
    ** Red - SCL Pin

*/

double latitude  = 0;
double longitude = 0;
String latbuf;
String lonbuf;
#define MotorTesting true

/*
  #define in1 22  // in1 Relay 1 is defined as Pin 22
  #define in2 24  // in2 Relay 1 is defined as Pin 24
  #define in3 26  // in3 Relay 1 is defined as Pin 26
  #define in4 28  // in4 Relay 1 is defined as Pin 28

  #define in5 23  // in1 Relay 2 is defined as Pin 23
  #define in6 25  // in2 Relay 2 is defined as Pin 25
  #define in7 27  // in3 Relay 2 is defined as Pin 27
  #define in8 29  // in4 Relay 2 is defined as Pin 29
*/

//#define PIN_OUTPUT_R 5 // Defining the PWM for the speed control of Right motor
//#define PIN_OUTPUT_L 4 // Defining the PWM for the speed control of Left Motor
//
//#define PIN_OUTPUT_F 7 // Defining the PWM for the speed control of Front Motor
//#define PIN_OUTPUT_B 6 // Defining the PWM for the speed control of Back Motor

///////////////////////////////////////// PID Definitions\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

#include <PID_v1.h>  // to be used later while doing PID control
#include <Servo.h>

// defining the pins of BLDC motors for speed & direction control.

byte m_f = 5;   /// front motor pin
byte m_r = 6;   /// right motor pin
byte m_l = 7;   /// left motor pin
byte m_a = 10;   /// Pseudo Back motor pin

byte m_k = 8;   /// back motor pin

Servo servo_f;
Servo servo_r;
Servo servo_l;
Servo servo_a;

Servo servo_b;

double yaw_adj;
double window = 3;   // Defining the window in degrees during which the motors will remain off (0+- 5)
double v_win = 0.05;  // Defining the window in meters for vertical motion control PID

int fwdspeed = 0;
double fwd_yaw = 0.0;


int vertical_speed = 0;     // Variable for storing the speed of motors for vertical motion. (0 lowest, 255 higest )
int cali_vert_speed = 255;  // Variable for storing the speed of motors for vertical motion (255 lowest, 0 highest)
int vert_control = 0;

int motion_control = 9;     // Variable for defining the motion of the motors.
int horizontal_speed = 0;   // Variable for storing horizontal speed
int cali_hori_speed = 255 ; // Variable for storing calibrated horizontal speed

int rotational_speed  = 0;
int cali_rot_speed    = 0;
int circular_speed    = 0;

int rot_angle = 0;

int r_speed = 0;
int l_speed = 0;
int r_cali  = 0;
int l_cali = 0;
/*
   0 = Default, Fixed position along the Z-axis,
   1 = Ability to Set the position to be stabilized along Z-axis.
   2 = Forward
   3 = Turn Right the control of speed
   4 = Turn Left
   5 = Rotate in a circle
   6 = Move Upwards (Without PID)
   7 = Move Downwards (Without PID)
   10 = Move forward (Without PID)
   11 = Move Backwards (Without PID)
   9 = STOP Everything

*/

///////////////////////// Yaw, Pitch & Roll //////////////////////////

/// PID Definitions///

double  pitch_1 = 0.0;
double  roll_1  = 0.0;
double  yaw_1   = 0.0;       // defining the variables for the pitch, roll and yaw angles
double  pitch_pid = 0.0, roll_pid = 0.0 , yaw_pid = 0.0;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;     // The variables of PID Control.
double readsetpoint = 0.0;
// explain here plz
double cali_output = 0;
//Specify the links and initial tuning parameters
double Kp = 0, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/// Vertical PID Definitions///
/////////////////\\\\\\\\\\\\\\\\\\\\

double Setpressure, Input_p, Output_p;
double Kpp = 0, Kip = 0, Kdp = 0;
double cali_vout = 0 , cali_vout2 = 0;
double depth_adj = 0;

PID vPID(&Input_p, &Output_p, &Setpressure, Kpp, Kip, Kdp, DIRECT);

////
double angle_adj = 0; // for storing the corrected angle for the case of stable movement.
double base_speed = 1700; // for storing the base speed in the case of stable movement.
double set_depth = 0;
double crc_depth = 0;

///// Water Pressure Sensors /////
#include "MS5837.h"
MS5837 p_sensor;
double p_temp = 0;
double p_pressure = 0;
double p_depth = 0;
double p_altitude = 0;


////////////////////////////////////////
///////////// GPS Module ///////////////
////////////////////////////////////////

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
float START_LAT, START_LON;
TinyGPSPlus gps;
//SoftwareSerial ss(RXPin, TXPin);
HardwareSerial & ss = Serial1;

///////////////////////////////////////
///////////// pH Sensor ///////////////
#define pHSensorPin A0           //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define LED 3
#define samplingInterval 200
#define printInterval 800


#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;
static float pHValue, voltage; // Variables that store the readings from the pH sensor.


///////////Bluetooth Module////////////
char junk;
String inputString = "";

/////////////SD Card Module///////////////
#include <SPI.h>
#include <SD.h>
#define FILE_BASE_NAME "Stat"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.txt";
File DataFile;
const int chipSelect = 53;

#define SDWriteinterval 250
///////////////////////////////////////
////////////***************////////////
//////////// Analog Sensors////////////
///////////////////////////////////////
/* Connections:
    A0 -> pH Meter
    A1 -> TDS Meter
    A2 -> Turbidity Sensor
    A6 -> Temperature Sensor

    Outputs:
    temp_cls = Temperature in Degree Celsius
    turbidity_cal = Turbidity in NTU
    tdsValue = TDS in ppm
    cond = Conductivity in micro siemens
*/
#define TdsSensorPin A1
#define TURBIDITYPIN A2
#define THERMISTORPIN A4
///////////// Thermistor /////////////

const int RunAvgTempCnt = 16;
float RunAvgBuftemp[RunAvgTempCnt];
int   NextRunAvgtemp ;
int   temp_analog;
int   temp_fht;
int   temp_cls;
const int RunAvgCnt = 16;

//////////////////////////////////////
///////////Turbidity Meter////////////

float turbidity_cal  = 0.0; /// Gives the Turbidity in NTU (Nephelometric Turbidity Unit)

//////////////////////////////////////
//////////// TDS Sensor //////////////

#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffertds[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemptds[SCOUNT];
int analogBufferIndextds = 0, copyIndextds = 0;
float averageVoltagetds = 0, tdsValue = 0, cond = 0;

/////////////////////////////////////
///////// Water Flow Meter //////////
byte sensorInterruptwf = 0;  // 0 = digital pin 2
byte sensorPinwf       = 2;

// The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.

float calibrationFactorwf = 4.5; // Defining a variable for the calibration factor
volatile byte pulseCountwf = 0.0;  // Variable for counting the pulse.

unsigned int frac;
float flowRate = 0;
unsigned int flowMilliLitres = 0;
unsigned long totalMilliLitres = 0;

unsigned long oldTimewf = 0;

////////////////////// Defining the variables for keeping a record of Time////////////////////
static unsigned long samplingTime = millis();
static unsigned long printTime    = millis();
static unsigned long sdTime       = millis();
////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////*******************\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


/*
  "9DOF Razor IMU" hardware versions: SEN-10125 and SEN-10736

  ATMega328@3.3V, 8MHz

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10125
  HMC5883L : Magnetometer on SEN-10736
  ITG-3200 : Gyro

  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8Mhz) w/ATmega328"
*/

/*
   website reference:
  "9DOF Sensor Stick" hardware versions: SEN-10183, SEN-10321 and SEN-10724

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10183 and SEN-10321
  HMC5883L : Magnetometer on SEN-10724
  ITG-3200 : Gyro
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.
  // comment here that how different boards have different seeting
  //e.g. one setting of ur current board
  // & one setting for sparcfun board... put link of purchase of sparkfun
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up

  Transformation order: first yaw then pitch then roll.
*/

/*
  Serial commands that the firmware understands:

  "#o<params>" - Set OUTPUT mode and parameters. The available options are:

      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.

      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).

      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.

      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).

      // Error message output
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.


  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.


  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.


  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)

  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.

  The status LED will be on if streaming output is enabled and off otherwise.

  Byte order of binary output is little-endian: least significant byte comes first.
*/



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)



// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 57600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// Bluetooth
// You can set this to true, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// When using this, streaming output will only be enabled as long as we're connected. That way
// receiver and sender are synchronzed easily just by connecting/disconnecting.
// It is not necessary to set this! It just makes life easier when writing code for
// the receiving side. The Processing test sketch also works without setting this.
// NOTE: When using this, OUTPUT__STARTUP_STREAM_ON has no effect!
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


////////////////////****************\\\\\\\\\\\\\\\\\\\\
// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
//accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX
//accel x,y,z (min/max) = -258/251  -276/254  -292/296
/*
   Calibration of IMU in free Space (Without mounting on AUV)
  #define ACCEL_X_MIN ((float) -266)
  #define ACCEL_X_MAX ((float) 254)
  #define ACCEL_Y_MIN ((float) -274)
  #define ACCEL_Y_MAX ((float) 245)
  #define ACCEL_Z_MIN ((float) -286)
  #define ACCEL_Z_MAX ((float) 222)
*/

//// calibration on auv June-2018
//#define ACCEL_X_MIN ((float) -276)
//#define ACCEL_X_MAX ((float) 282)
//#define ACCEL_Y_MIN ((float) -281)
//#define ACCEL_Y_MAX ((float) 268)
//#define ACCEL_Z_MIN ((float) -306)
//#define ACCEL_Z_MAX ((float) 226)


// Calibration on AUV, done on 3-9-18
//#define ACCEL_X_MIN ((float) -276)
//#define ACCEL_X_MAX ((float) 291)
//#define ACCEL_Y_MIN ((float) -306)
//#define ACCEL_Y_MAX ((float) 286)
//#define ACCEL_Z_MIN ((float) -311)
//#define ACCEL_Z_MAX ((float) 230)

// Calibration on AUV, done on 10-5-19
#define ACCEL_X_MIN ((float) -281)
#define ACCEL_X_MAX ((float) 288)
#define ACCEL_Y_MIN ((float) -290)
#define ACCEL_Y_MAX ((float) 276)
#define ACCEL_Z_MIN ((float) -292)
#define ACCEL_Z_MAX ((float) 217)



// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
//#define MAGN_X_MIN ((float) -600)
//#define MAGN_X_MAX ((float) 600)
//#define MAGN_Y_MIN ((float) -600)
//#define MAGN_Y_MAX ((float) 600)
//#define MAGN_Z_MIN ((float) -600)
//#define MAGN_Z_MAX ((float) 600)


/// Calibration done on 3-9-18

#define MAGN_X_MIN ((float) -448)
#define MAGN_X_MAX ((float) 583)
#define MAGN_Y_MIN ((float) 26)
#define MAGN_Y_MAX ((float) 1017)
#define MAGN_Z_MIN ((float) -584)
#define MAGN_Z_MAX ((float) 255)


// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// First Calibration (Lab)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {297.662, -478.737, -532.150};
//const float magn_ellipsoid_transform[3][3] = {{0.590171, -0.111656, 0.207015}, {-0.111656, 0.520205, -0.193718}, {0.207015, -0.193718, 0.756217}};

// Second Calibration (Home)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {57.7658, 97.8903, 151.901};
//const float magn_ellipsoid_transform[3][3] = {{0.921984, -0.0179129, -0.00725522}, {-0.0179129, 0.935006, -0.000411116}, {-0.00725522, -0.000411116, 0.999299}};

// Third Calibration (AUV - First Attempt)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {248.354, 722.333, -152.534};
//const float magn_ellipsoid_transform[3][3] = {{0.771350, 0.0513129, 0.0184901}, {0.0513129, 0.974633, -0.0541056}, {0.0184901, -0.0541056, 0.818330}};

// Fourth Calibration
//
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {153.048, 632.770, -242.900};
//const float magn_ellipsoid_transform[3][3] = {{0.771290, 0.0302069, -0.0601656}, {0.0302069, 0.972017, -0.0745118}, {-0.0601656, -0.0745118, 0.700786}};

///// Calibration done on 3-9-18
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {101.967, 517.346, -204.896};
//const float magn_ellipsoid_transform[3][3] = {{0.673644, 0.00790633, 0.00887635}, {0.00790633, 0.865715, 0.0172984}, {0.00887635, 0.0172984, 0.997471}};

/// Calibration done on 10-5-19
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {4.29216, 220.209, 171.593};
const float magn_ellipsoid_transform[3][3] = {{0.913953, 0.0549884, -0.0304703}, {0.0549884, 0.737711, 0.0505871}, { -0.0304703, 0.0505871, 0.984948}};



// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
/*
  #define GYRO_AVERAGE_OFFSET_X ((float) 63.73)
  #define GYRO_AVERAGE_OFFSET_Y ((float) 24.46)
  #define GYRO_AVERAGE_OFFSET_Z ((float) -1.2)
*/
// The following calculated from AUV
//#define GYRO_AVERAGE_OFFSET_X ((float) 72.15)
//#define GYRO_AVERAGE_OFFSET_Y ((float) 27.5)
//#define GYRO_AVERAGE_OFFSET_Z ((float) -1.2)


///// Calibration done on 3-9-18
//
//#define GYRO_AVERAGE_OFFSET_X ((float) 73.73)
//#define GYRO_AVERAGE_OFFSET_Y ((float) 23.61)
//#define GYRO_AVERAGE_OFFSET_Z ((float) 1.14)

/// Calibration done on 10-5-19
#define GYRO_AVERAGE_OFFSET_X ((float) 76.73)
#define GYRO_AVERAGE_OFFSET_Y ((float) 29.11)
#define GYRO_AVERAGE_OFFSET_Z ((float) -4.62)


/*
  // Calibration example:

  // "accel x,y,z (min/max) = -277.00/264.00  -256.00/278.00  -299.00/235.00"
  #define ACCEL_X_MIN ((float) -277)
  #define ACCEL_X_MAX ((float) 264)
  #define ACCEL_Y_MIN ((float) -256)
  #define ACCEL_Y_MAX ((float) 278)
  #define ACCEL_Z_MIN ((float) -299)
  #define ACCEL_Z_MAX ((float) 235)

  // "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
  //#define MAGN_X_MIN ((float) -511)
  //#define MAGN_X_MAX ((float) 581)
  //#define MAGN_Y_MIN ((float) -516)
  //#define MAGN_Y_MAX ((float) 568)
  //#define MAGN_Z_MIN ((float) -489)
  //#define MAGN_Z_MAX ((float) 486)

  // Extended magn
  #define CALIBRATION__MAGN_USE_EXTENDED true
  const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
  const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};

  // Extended magn (with Sennheiser HD 485 headphones)
  //#define CALIBRATION__MAGN_USE_EXTENDED true
  //const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
  //const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};

  //"gyro x,y,z (current/average) = -40.00/-42.05  98.00/96.20  -18.00/-18.36"
  #define GYRO_AVERAGE_OFFSET_X ((float) -42.05)
  #define GYRO_AVERAGE_OFFSET_Y ((float) 96.20)
  #define GYRO_AVERAGE_OFFSET_Z ((float) -18.36)
*/


// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/


// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
// Generate compile error
#error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 3  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3] = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; // Omega Integrator
float Omega[3] = {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;




void read_sensors()
{
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  // GET ROLL
  // Compensate pitch of gravity vector
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors()
{
  // Compensate accelerometer error
  accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

  // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
  for (int i = 0; i < 3; i++)
    magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
  Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
  magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

  // Compensate gyroscope error
  gyro[0] -= GYRO_AVERAGE_OFFSET_X;
  gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
  gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;

  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}




/*
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SETUP
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/





void setup()
{

  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);


  //------GPS---------//
  Serial1.begin(GPSBaud);
  ss.begin(GPSBaud);    /// GPS is connected to Serial 1
  //------------------//


  //-----Bluetooth----//
  Serial2.begin(9600);  /// Bluetooth is connected to Serial 2
  //------------------//


  //----Flow Meter----//
  pinMode(sensorPinwf, INPUT);
  digitalWrite(sensorPinwf, HIGH);
  attachInterrupt(sensorInterruptwf, pulseCounter, FALLING);
  //------------------//

  //-----Water Pressure Sensor -----//

  Wire.begin();
  p_sensor.init();
  p_sensor.setFluidDensity(997);
  //--------------------------------//


  //-----SD Card -----//
  // Initializing SD Card

  if (SD.begin(chipSelect))

  {
    Serial.println("Card initialized successfully");
    // don't do anything more:
    delay (100);
  }

  else
  {
    Serial.println( " Error, SD Card could not be initalized ");
  }

  ////////////////////////////////////////
  ////// Finding an unused file name//////

  if (BASE_NAME_SIZE > 6)
  {
    //error("FILE_BASE_NAME too long");
  }
  while (SD.exists(fileName))
  {
    if (fileName[BASE_NAME_SIZE + 1] != '9')
    {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9')
    {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else
    {
      Serial.println("Can't Create File name");
    }
  }

  ////////////////////////////////////////////////////////

  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();

  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  ////////////////////////////////////////// ESC motors setup \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//  servo_k.attach(m_k);
  servo_f.attach(m_f);
  servo_r.attach(m_r);
  servo_l.attach(m_l);
  servo_a.attach(m_a);
  
//  servo_k.writeMicroseconds(1500); // send "stop" signal to ESC.
  servo_f.writeMicroseconds(1500);
  servo_r.writeMicroseconds(1500);
  servo_l.writeMicroseconds(1500);
  servo_a.writeMicroseconds(1500);

  delay(4000); // delay to allow the ESC to recognize the stopped signal

  /////////////// PID Controller Setup (Horizontal) \\\\\\\\\\\\\\\\

  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);     //turn the PID on
  Input =  yaw_1;           //initialize the variables we're linked to

  /////////////// PID Controller Setup (Vertical) \\\\\\\\\\\\\\\\
  Setpressure = 0 ;
  vPID.SetMode(AUTOMATIC);
  Input_p = p_depth;

  ///////////////////////////////////// **************** \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif

#if (MotorTesting == true)
  {

    //servo_k.writeMicroseconds(1500); // send "stop" signal to ESC.
    servo_a.writeMicroseconds(1500);
    servo_f.writeMicroseconds(1500);
    servo_r.writeMicroseconds(1500);
    servo_l.writeMicroseconds(1500);


    delay (1000);

//    servo_k.writeMicroseconds(2000); // send "Maximum Positive" signal to ESC.
    servo_a.writeMicroseconds(2000);
    servo_f.writeMicroseconds(2000);
    servo_r.writeMicroseconds(1500);
    servo_l.writeMicroseconds(1500);
    
    delay (1000);

//    servo_k.writeMicroseconds(1500); // send "stop" signal to ESC.
    servo_a.writeMicroseconds(1500);
    servo_f.writeMicroseconds(1500);
    servo_r.writeMicroseconds(2000);
    servo_l.writeMicroseconds(2000);
    
    delay (1000);

//    servo_k.writeMicroseconds(1200); // send "stop" signal to ESC.

    servo_a.writeMicroseconds(1200);
    servo_f.writeMicroseconds(1200);
    servo_r.writeMicroseconds(1500);
    servo_l.writeMicroseconds(1500);  
    
    delay (1000);

//    servo_k.writeMicroseconds(1500); // send "stop" signal to ESC.

    servo_a.writeMicroseconds(1500);
    servo_f.writeMicroseconds(1500);
    servo_r.writeMicroseconds(1200);
    servo_l.writeMicroseconds(1200);

    delay (1000);

//    servo_k.writeMicroseconds(1500); // send "stop" signal to ESC.
    servo_a.writeMicroseconds(1500);
    servo_f.writeMicroseconds(1500);
    servo_r.writeMicroseconds(1500);
    servo_l.writeMicroseconds(1500);

    delay (1000);


    Serial.print("Initialization Completed. ");

  }
#endif

}

//////////////////////////
//////////////////////////
/////// Main loop ////////
//////////////////////////
//////////////////////////

void loop()
{
  /////////////////////////

  ////////////////////////

  // Read incoming control messages

  if (Serial.available() >= 2)
  {
    if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands
      if (command == 'f') // request one output _f_rame
        output_single_on = true;
      else if (command == 's') // _s_ynch request
      {
        // Read ID
        byte id[2];
        id[0] = readChar();
        id[1] = readChar();

        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
      else if (command == 'o') // Set _o_utput mode
      {
        char output_param = readChar();
        if (output_param == 'n')  // Calibrate _n_ext sensor
        {
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 't') // Output angles as _t_ext
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_TEXT;
        }
        else if (output_param == 'b') // Output angles in _b_inary format
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == 'c') // Go to _c_alibration mode
        {
          output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 's') // Output _s_ensor values
        {
          char values_param = readChar();
          char format_param = readChar();
          if (values_param == 'r')  // Output _r_aw sensor values
            output_mode = OUTPUT__MODE_SENSORS_RAW;
          else if (values_param == 'c')  // Output _c_alibrated sensor values
            output_mode = OUTPUT__MODE_SENSORS_CALIB;
          else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
            output_mode = OUTPUT__MODE_SENSORS_BOTH;

          if (format_param == 't') // Output values as _t_text
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b') // Output values in _b_inary format
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == '0') // Disable continuous streaming output
        {
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        }
        else if (output_param == '1') // Enable continuous streaming output
        {
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        }
        else if (output_param == 'e') // _e_rror output settings
        {
          char error_param = readChar();
          if (error_param == '0') output_errors = false;
          else if (error_param == '1') output_errors = true;
          else if (error_param == 'c') // get error count
          {
            Serial.print("#AMG-ERR:");
            Serial.print(num_accel_errors); Serial.print(",");
            Serial.print(num_magn_errors); Serial.print(",");
            Serial.println(num_gyro_errors);
          }
        }
      }

      // else if (command ==

      ///////////////////////////////////////////////////////////////////////////

#if OUTPUT__HAS_RN_BLUETOOTH == true
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
        turn_output_stream_on();
      else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
        turn_output_stream_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
    }
    else
    { } // Skip character
  }




  // Time to read the sensors again?
  if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();  // Check if this session needs a reset
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {
      // Apply sensor calibration
      compensate_sensor_errors();

      // Run DCM algorithm
      Compass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();

      if (output_stream_on || output_single_on) output_angles();
    }
    else  // Output sensor values
    {
      if (output_stream_on || output_single_on) output_sensors();
    }

    output_single_on = false;

#if DEBUG__PRINT_LOOP_TIME == true
    Serial.print("loop time (ms) = ");
    Serial.println(millis() - timestamp);
#endif
  }
#if DEBUG__PRINT_LOOP_TIME == true
  else
  {
    Serial.println("waiting...");
  }
#endif



  /////////////////// Calling Functions \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


  //bluetooth ();    //// Bluetooth Control Is yet to be integrated into the Code, for controlling the AUV and for tuning the PID


  /////////////////////////////////// PID Control Loop \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


  /*
    Input = pitch_1;
    myPID.Compute();
    analogWrite(PIN_OUTPUT, Output);

    Serial.print("Output")  ;   Serial.print(Output);   Serial.print(" , ");
    Serial.print("Pitch: ") ;   Serial.print(pitch_1) ;   Serial.print(" , ");
    Serial.print("Roll: ")  ;   Serial.print(roll_1)  ;   Serial.print(" , ");
    Serial.print(" Yaw: ")  ;   Serial.print(yaw_1) ; Serial.println(" , ");

  */

  bluetooth_control();
  PID_control();

  if (millis() - samplingTime > samplingInterval)
  {
    gp();   /// Calling the function that fetches the readings from the GPS Sensor.
    pH();   /// Calling the function that fetches the readings from the pH Sensor.
    temperature ();
    turbidity();
    TDS();
    water_flow();
    samplingTime = millis();
  }
  p_sens();
  sd_data();

}


/////////// The Below Code contains the functions for the waterflow meter //////////


void water_flow()

{

  if ((millis() - oldTimewf) > 1000)   // Only process counters once per second

  {
    // Disable the interrupt while calculating flow rate and sending the value to
    // the host
    detachInterrupt(sensorInterruptwf);

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - oldTimewf)) * pulseCountwf) / calibrationFactorwf;

    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTimewf = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;

    // Determine the fractional part. The 10 multiplier gives us 1 decimal place.
    frac = (flowRate - int(flowRate)) * 10;

    // Reset the pulse counter so we can start incrementing again
    pulseCountwf = 0;

    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(sensorInterruptwf, pulseCounter, FALLING);
    ///// Flow in Millilitres per second, the flow rate is being stored in flowMilliLitres
    ///// Cumulative total of litres flowed are stored in totalMilliLitres

  }

}

/*
  Interrupt Service Routine
*/
void pulseCounter()
{
  // Increment the pulse counter
  pulseCountwf++;
}



