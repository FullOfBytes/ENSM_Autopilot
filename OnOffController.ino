/* 
This code is based on the excellent code of LINGIB :
------------------------------------------------	
	Tilt Compensated Compass
    ------------------------
    "tilt_comp_compass.ino"
    Code by LINGIB
    https://www.instructables.com/member/lingib/instructables/
    Last update 16 September 2019
------------------------------------------------
Modified and used by Autopilot Team At ENSM, Thank You Very Much LINGIB.


   -------------
   TERMS OF USE:
   -------------
   The software is provided "AS IS", without any warranty of any kind, express or implied,
   including but not limited to the warranties of mechantability, fitness for a particular
   purpose and noninfringement. In no event shall the authors or copyright holders be liable
   for any claim, damages or other liability, whether in an action of contract, tort or
   otherwise, arising from, out of or in connection with the software or the use or other
   dealings in the software.

   -------------
   WARNING:
   -------------
   Do NOT use this compass in situations involving safety to life such as navigation at sea.
*/

/**/


// ----- Libraries
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Servo.h>


#define IN3 5
#define IN4 6
#define ENB 3


// ----- Gyro
#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250 
#define MPU9250_I2C_master_enable 0x6A                                  // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN

#define Frequency 50                                                     // 20 mS sample interval 
#define Sensitivity 65.5                                                // Gyro sensitivity (see data sheet)

#define Sensor_to_deg 1/(Sensitivity*Frequency)                         // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg*DEG_TO_RAD                          // Convert sensor reading to radians
#define g 9.81
#define g_to_mps2 g/4096

#define Loop_time 1000000/Frequency                                     // Loop time (uS)
long    Loop_start;                                                     // Loop start time (uS)

#define LedPin 6


//----------------- GPS and SoftwareSerial
// We are not using the GPS module !! But we let all the code for future use, You are free to delete it.
#define RXPin 9       // TX on GPS
#define TXPin 10      // RX on GPS                               
static const uint32_t GPSBaud = 9600; // Software Serial Speed To talk with the GPS Module
float MyLoc_Lat = 0.0;
float MyLoc_Lng = 0.0;


int Confirmation = 0;
float cible_actuelle[2] = {0.0, 0.0}; // Target Coordinates Lat, Long
// Our Parcour has just 3 points as max, Add entries if you want more *_*.

float parcour[6] = {36.65024839882367, 2.696118238404322, 36.64994174948469, 2.6960511831810936, 36.64986434736686, 2.6965541005283713};
/*
36.65024839882367, 2.696118238404322      ENSM1
36.64994174948469, 2.6960511831810936     ENSM2
36.64986434736686, 2.6965541005283713     ENSM3 
*/

float Bearing           = 0;
float Distance          = 0;
float Dsitance_Traveled = 0;
float RelativeBearing   = 0;

float Loc_Lat = 0;
float Loc_Lng = 0;
int target_done = 0;
unsigned long GPS_Lock_Timer = 0.0;
unsigned long GPS_Timeout    = 0.0;
unsigned long Serial_Timer   = 0.0;
bool SerialTimeout = false;

TinyGPSPlus gps;
Servo GouverSVO;

//----------- Location -------------
// We are using GPS data coming from an Android app on a smartphone
String inputString = "";         // a String to hold incoming data from Serial.
bool stringComplete = false;     // whether the string is complete.
float MyLocation[] = {0.0, 0.0}; // put GPS Coordinates Here.
float GPS_speed    = 0.0;        // put GPS Speed Here.


// ----------- RF 24

const byte address [][6] = {"00001", "00002"};  // planning to implement a 2 ways comm in future.
typedef struct
{
  float Loc_Lat = 0.0;
  float Loc_Lng = 0.0;
  float   speed = 0.0;
  float RelativeBearing = 0.0;
  float   pitch = 0.0;
  float   yaw   = 0.0;
  float   roll  = 0.0;
  byte    flags = 0x0;
} Info_Struct;
Info_Struct Infos;

//--------- RF Object
RF24 radio(7, 8);   // nRF24L01 (CE, CSN)
// 13 SCK
// 12 MISO
// 11 MOSI

// ------------ Gyroscope
int     Gyro_x,     Gyro_y,     Gyro_z;
long    Gyro_x_cal, Gyro_y_cal, Gyro_z_cal;
float   Gyro_pitch, Gyro_roll, Gyro_yaw;
float   Gyro_pitch_output, Gyro_roll_output;

// ----- Accelerometer
long    Accel_x,      Accel_y,      Accel_z,    Accel_total_vector;
long    Accel_x_cal, Accel_y_cal;
float   Accel_pitch,  Accel_roll;

// ----- Magnetometer
#define AK8963_I2C_address 0x0C                                             // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A                                             // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                            // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                   // Data ready mask
#define AK8963_overflow_mask 0b00001000                                     // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                    // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10                                                // X,Y,Z fuse ROM

// ----- Compass heading
/*
  The magnetic declination for Bousmail, Algiers is +1.5 degrees
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  Uncomment the declination code within the main loop() if you want True North.
*/
float   Declination = +1.5;                                                 //  Degrees ... replace this declination with yours
float     Heading;

int     Mag_x,                Mag_y,                Mag_z;                  // Raw magnetometer readings
float   Mag_x_dampened,       Mag_y_dampened,       Mag_z_dampened;
float   Mag_x_hor, Mag_y_hor;
float   Mag_pitch, Mag_roll;

// ----- Record compass offsets, scale factors, & ASA values
/*
   These values seldom change ... an occasional check is sufficient
   (1) Open your Arduino "Serial Monitor
   (2) Set "Record_data=true;" then upload & run program.
   (3) Replace the values below with the values that appear on the Serial Monitor.
   (4) Set "Record_data = false;" then upload & rerun program.
*/
bool    Record_data = false;
int     Mag_x_offset = 245,      Mag_y_offset = 171,     Mag_z_offset = -170;   // Hard-iron offsets
float   Mag_x_scale = 1.14,     Mag_y_scale = 0.95,     Mag_z_scale = 0.94;    	// Soft-iron scale factors
float   ASAX = 1.20,            ASAY = 1.20,            ASAZ = 1.16;           	// (A)sahi (S)ensitivity (A)djustment fuse ROM values.


// ----- Flags
bool Gyro_synchronised = false;
bool Flag = false;

// ----- Debug
#define Switch A0                       // Connect an SPST switch between A0 and GND to enable/disable tilt stabilazation
long Loop_start_time;
long Debug_start_time;


// -----------------
//  Setup
// -----------------
void setup()
{

  // Put the first Target position of parcour on cible_actuelle.
  target_done = 0;
  Confirmation = 0;
  cible_actuelle[0] = parcour[target_done];
  cible_actuelle[1] = parcour[target_done + 1];
  

  pinMode(ENB, OUTPUT); // Motor Speed PWM pin.
  pinMode(IN3, OUTPUT); // Motor + pin.
  pinMode(IN4, OUTPUT); // Motor - pin.
  
  inputString.reserve(30);
  Infos.Loc_Lat = Loc_Lat = MyLoc_Lat;
  Infos.Loc_Lng = Loc_Lng = MyLoc_Lng;

  // ----- Serial communication
  Serial.begin(9600);                                 //Use only for debugging
  Wire.begin();                                         //Start I2C as master
  Wire.setClock(400000);
  // nRF24 Init ----------------------------

  radio_init();



  // Attach The Servo
  //---------------------------
   GouverSVO.attach(9);
  //------------ Servo End ----




  //-----------Configure the magnetometer---------------
   configure_magnetometer();
  //----------- Config Mag End -------------------------


  //--------------------Calibrate the magnetometer------------------
  /*
     Calibrate only needs to be done occasionally.
     Enter the magnetometer values into the "header"
     then set "Record_data = false".
  */
  if (Record_data == true)
  {
    calibrate_magnetometer();
  }
//------------------- Mag Calibration End --------------------------


 //---------------- Configure the gyro -----------
  config_gyro();
 //---------------- Gyro Config  End -------------

//--------------- Calibrate Gyro and Accelerometer ----------------
  calibrate_gyro();
  calibrate_Accel();
//--------------- Cal Gyro and Accel End --------------------------
 
  //Initial State Engine Stopped and Rudder at 90°
  Engine(0);
  GouverSVO.write(90);

    

  Debug_start_time = micros();                          // Controls data display rate to Serial Monitor
  Loop_start_time  = micros();                           // Controls the Gyro refresh rate
}

// ----------------------------
// Main loop
// ----------------------------
void loop()
{
  unsigned long  debut = millis();
    
  if  ((millis() - Serial_Timer) > 3000) {
    // We Lost Serial Communication
    Infos.Loc_Lat = Loc_Lat = 0.0;
    Infos.Loc_Lng = Loc_Lng = 0.0;
    Infos.RelativeBearing   = 0.0;
    Infos.speed = 0.0;
    bitSet(Infos.flags, 0);
  }

  if (Infos.flags & 0x2 ) {
    Serial.println("RF24: Write Failed");
    if (Infos.flags & 0x4) {
      Serial.println("RF24: Hardware not detected");
    }
    radio.powerDown();
    radio_init();
  }




  ////////////////////////////////////////////
  //        PITCH & ROLL CALCULATIONS       //
  ////////////////////////////////////////////

  /*
     --------------------
     MPU-9250 Orientation
     --------------------
     Component side up
     X-axis facing forward
  */



  // ----- read the raw accelerometer and gyro data --------------
  read_mpu_6050_data();                                             // Read the raw acc and gyro data from the MPU-6050




  // ----- Adjust Gyro for offsets ----------------
  Gyro_x -= Gyro_x_cal;                                             // Subtract the offset from the raw gyro_x value
  Gyro_y -= Gyro_y_cal;                                             // Subtract the offset from the raw gyro_y value
  Gyro_z -= Gyro_z_cal;                                             // Subtract the offset from the raw gyro_z value



  // ----- Adjust Accel for Offsets ---------------
  Accel_x = Accel_x - Accel_x_cal;
  Accel_y = Accel_y - Accel_y_cal;



  // ----- Calculate travelled angles
  /*
    ---------------------------
    1.1 : Adjust Gyro_xyz signs for:
    ---------------------------
    Pitch (Nose - up) = +ve reading = - Right Hand Pitch (sensor)
    Roll (Right - wing down) = +ve reading = + Right Hand Roll (sensor)
    Yaw (Clock - wise rotation)  = +ve reading = - Right Hand Yaw (sensor)
  */

  /*
    -------------------------
    1.2 : Gyro angle calculations:
    -------------------------
    Gyro gives : raw value of 65.5 = 1 deg/s , 65.5 is Sensetivity Scale
    Frequency of getting Values is 250Hz --> every T=1/250 = 0.004sec
    So to get the degrees travled :
    example of raw value of 393 :
    393 / 65.5 = 6 deg/s
    But we take a value each 0.004Sec :
    6 deg  ---> 1sec
    x deg  ---> 0.004 sec  --> x = 6 * 0.004 = 0.024 deg travled in 0.004 sec

    SO : degrees_travled = (raw_value / Sensetivity) * (1 / Reading Frequency) = raw_value / (Sensitivity * Frequency)
    Here Sensor_to_deg = 1 / (Sensetivity * Frequency)
  */

  Gyro_pitch += -Gyro_y * Sensor_to_deg;                            // Integrate the raw Gyro_y readings
  Gyro_roll += Gyro_x * Sensor_to_deg;                              // Integrate the raw Gyro_x readings
  Gyro_yaw += -Gyro_z * Sensor_to_deg;                              // Integrate the raw Gyro_z readings




  // ----- Compensate pitch and roll for gyro yaw  ----- Reste a Devlopper ??????
  Gyro_pitch += Gyro_roll * sin(Gyro_z * Sensor_to_rad);            // Transfer the roll angle to the pitch angle if the Z-axis has yawed
  Gyro_roll -= Gyro_pitch * sin(Gyro_z * Sensor_to_rad);            // Transfer the pitch angle to the roll angle if the Z-axis has yawed

  // ----- Accelerometer angle calculations
  Accel_total_vector = sqrt((Accel_x * Accel_x) + (Accel_y * Accel_y) + (Accel_z * Accel_z));   // Calculate the total (3D) vector
  Accel_pitch = asin((float)Accel_x / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the pitch angle
  Accel_roll = asin((float)Accel_y / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the roll angle



  // ----- Zero any residual accelerometer readings
  /*
     Place the accelerometer on a level surface
     Adjust the following two values until the pitch and roll readings are zero
  */
  Accel_pitch -= -0.2f;                                             //Accelerometer calibration value for pitch
  Accel_roll -= 1.1f;                                               //Accelerometer calibration value for roll

  // ----- Use Complementary Filtre (Gyro & Acc) to Correct for any gyro drift
  if (Gyro_synchronised)
  {
    // ----- Gyro & accel have been synchronised
    Gyro_pitch = Gyro_pitch * 0.9996 + Accel_pitch * 0.0004;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    Gyro_roll = Gyro_roll * 0.9996 + Accel_roll * 0.0004;           //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {
    // This code is executed just once, the First loop, just to fixe the
    // initial conditions for Gyro_Pitch and Gyro_Roll
    // ----- Synchronise gyro & accel
    Gyro_pitch = Accel_pitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll = Accel_roll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_synchronised = true;                                             //Set the IMU started flag
  }

  // ----- Dampen the pitch and roll angles (Filtre pass bas pour filtrer les piques)
  Gyro_pitch_output = Gyro_pitch_output * 0.9 + Gyro_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  Gyro_roll_output  = Gyro_roll_output * 0.9 + Gyro_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value





  ////////////////////////////////////////////
  //        MAGNETOMETER CALCULATIONS       //
  ////////////////////////////////////////////
  /*
     --------------------------------
     Instructions for first time use
     --------------------------------
     Calibrate the compass for Hard-iron and Soft-iron
     distortion by temporarily setting the header to read
     bool    Record_data = true;

     Turn on your Serial Monitor before uploading the code.

     Slowly tumble the compass in all directions until a
     set of readings appears in the Serial Monitor.

     Copy these values into the appropriate header locations.

     Edit the header to read
     bool    Record_data = false;

     Upload the above code changes to your Arduino.

     This step only needs to be done occasionally as the
     values are reasonably stable.
  */

  //----- Read the magnetometer
  read_magnetometer();

  //----- Fix the pitch, roll, & signs
  /*
     MPU-9250 gyro and AK8963 magnetometer XY axes are orientated 90 degrees to each other
     which means that Mag_pitch equates to the Gyro_roll and Mag_roll equates to the Gryro_pitch

     The MPU-9520 and AK8963 Z axes point in opposite directions
     which means that the sign for Mag_pitch must be negative to compensate.
  */
  Mag_pitch = -Gyro_roll_output * DEG_TO_RAD;
  Mag_roll = Gyro_pitch_output * DEG_TO_RAD;

  // ----- Apply the standard tilt formulas
  Mag_x_hor = Mag_x * cos(Mag_pitch) + Mag_y * sin(Mag_roll) * sin(Mag_pitch) - Mag_z * cos(Mag_roll) * sin(Mag_pitch);
  Mag_y_hor = Mag_y * cos(Mag_roll) + Mag_z * sin(Mag_roll);

  // ----- Disable tilt stabization if switch closed
  if (!(digitalRead(Switch)))
  {
    // ---- Test equations
    Mag_x_hor = Mag_x;
    Mag_y_hor = Mag_y;
  }

  // ----- Dampen any data fluctuations
  Mag_x_dampened = Mag_x_dampened * 0.9 + Mag_x_hor * 0.1;
  Mag_y_dampened = Mag_y_dampened * 0.9 + Mag_y_hor * 0.1;

  // ----- Calculate the heading
  Heading = atan2(Mag_x_dampened, Mag_y_dampened) * RAD_TO_DEG;  // Magnetic North

  /*
     By convention, declination is positive when magnetic north
     is east of true north, and negative when it is to the west.
  */

  Heading += Declination;               // Geographic North
  if (Heading > 360.0) Heading -= 360.0;
  if (Heading < 0.0) Heading += 360.0;

  // ----- Allow for under/overflow
  if (Heading < 0) Heading += 360;
  if (Heading >= 360) Heading -= 360;




  // ---------------- Get Location if Available from Phone ----------------
  if (stringComplete) {

    Infos.Loc_Lat = Loc_Lat = MyLocation[0];
    Infos.Loc_Lng = Loc_Lng = MyLocation[1];
    Infos.speed = GPS_speed;
    inputString = "";
    stringComplete = false;

  }


  if ( (Infos.Loc_Lat != 0.0) && (Infos.Loc_Lng != 0.0) && !(Infos.flags & 0x1))
    // GPS coordinates available and Serial Comm with Phone is OK.
  {

    // Calculate Bearing From GPS Coordinates
    Bearing  = gps.courseTo(Loc_Lat, Loc_Lng, cible_actuelle[0], cible_actuelle[1]);
    Distance = gps.distanceBetween(Loc_Lat, Loc_Lng, cible_actuelle[0], cible_actuelle[1]);
    // Calculate RelativeBearing
    RelativeBearing = Bearing - Heading;
    Infos.RelativeBearing = RelativeBearing;
  } else {
    // Stop_Engine();
    // GouverSVO.write(90);

  }

  // ----- Display Heading, Pitch, and Roll
  //  Serial.print("Heading : ");
  //  Serial.print(Heading);
  //  Serial.print("  Pitch : ");
  //  Serial.print(Gyro_pitch_output);
  //  Serial.print("  Roll : ");
  //  Serial.println(Gyro_roll_output);



  Infos.yaw     = Heading;
  Infos.pitch   = Gyro_pitch_output;
  Infos.roll    = Gyro_roll_output;


  // radio.write if failed will delay for 100ms which is very long for our sampling time , all our values will be wrong.
  // So, You should take that in account.
  // There is another alternative radio.writeFast but you will not know if the Transmission is failed or not.
  // Keep Searching ********************
  if ( !radio.write(&Infos, sizeof(Infos)))
  {
    //Serial.println(" Babor Problem Sending!!");
    // Set the Second bit of flags to indicate Transmitting problem
    bitSet(Infos.flags, 1);
    if (!radio.isChipConnected())
    {
      // Set 3rd bit Indicating absence of nRF24
      bitSet(Infos.flags, 2);
    }
  } else {
    //Serial.println("Packet Sent Success");
    if ((Infos.flags & 0x2) || (Infos.flags & 0x4))
    {
      // if Transmitting is Ok, Just Clear all nRF24 Bits (no problemo *_*)
      bitClear(Infos.flags, 1);
      bitClear(Infos.flags, 2);
    }
  }


    
  //------------------------------ We have All Data We need to make Safe Navigation, Let's Go !!---------------------

  Go(Bearing, Infos.yaw , Infos.RelativeBearing, Distance, Infos.flags);

  //-----------------------------------------------------------------------------------------------------------------




  // -------------------------Loop control: We need to adjust and fix the loop's time for our integrations calculations.
  // We choose a 50Hz sampling Frequency, means 20 ms  (20000 us) Sampling time.
  
   unsigned long fin = millis() - debut;  // We should Verify the Loop's time
  
  /*
     Adjust the loop count for 20ms
  */
  while ((micros() - Loop_start_time) < 20000);
  
  Loop_start_time = micros(); // The start time for the next  loop.

  //Serial.println(fin); // Verify !! Are we meeting our 20000us condition ?

  // We should mention that the radio.write() function when failed takes a delay of 100ms to try to solve the problem,
  // that will affect the loop's time (100ms + loop time), So You will get wrong integrations angles (Roll,pitch,yaw), Be carful !!!

  // --------------------------------------------------------LOOP END --------------------------------------------------------------------------
}

/*
==================================================================
  Functions Declarations
==================================================================
*/

 //Serial Event For recieving Location from Phone
void serialEvent() {

  bitClear(Infos.flags, 0);
  Serial_Timer = millis();
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '/') {
      MyLocation[0] = inputString.toFloat();
      inputString = "";
    }

    if (inChar == ':') {
      MyLocation[1] = inputString.toFloat();
      inputString = "";
    }


    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:

    if (inChar == '\n') {
      GPS_speed = inputString.toFloat();
      stringComplete = true;
    }
  }
}

// RF24 initialization

void radio_init() {

  radio.begin();                         // StartUp Radio.
  radio.setAutoAck(false);                // No Ack, But we should implemented it for insurance.
  radio.setPALevel(RF24_PA_LOW);          // We choose Low for testing (Short Distance, about 1 meter)
  radio.openWritingPipe(address[1]);      // Address to use to transmit
  radio.openReadingPipe(1, address[0]);   // Addresse to use for recieving.
  radio.stopListening();                // Transmitter Mode..


}




// ----------------------------
//  Configure magnetometer
// ----------------------------
void configure_magnetometer()
{
  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-6050 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-through mode. To do this we must:
      - disable the MPU-9250 slave I2C and
      - enable the MPU-9250 interface bypass mux
  */
  // ----- Disable MPU9250 I2C master interface
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);                            // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                                                 // Disable the I2C master interface
  Wire.endTransmission();

  // ----- Enable MPU9250 interface bypass mux
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);                  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                                 // Enable the bypass mux
  Wire.endTransmission();

  // ----- Access AK8963 fuse ROM
  /* The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // CNTL[3:0] mode bits
  Wire.write(0b00011111);                                           // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Get factory XYZ sensitivity adjustment values from fuse ROM
  /* There is a formula on page 53 of "MPU-9250, Register Map and Decriptions, Revision 1.4":
      Hadj = H*(((ASA-128)*0.5)/128)+1 where
      H    = measurement data output from data register
      ASA  = sensitivity adjustment value (from fuse ROM)
      Hadj = adjusted measurement data (after applying
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                                      // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);                          // Request 3 bytes of data
  while (Wire.available() < 3);                                     // Wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;                       // Adjust data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  // ----- Power down AK8963 while the mode is changed
  /*
     This wasn't necessary for the first mode change as the chip was already powered down
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00000000);                                           // Set mode to power down
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Set output to mode 2 (16-bit, 100Hz continuous)
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00010110);                                           // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change
}

// -------------------------------
//  Calibrate magnetometer
// -------------------------------
void calibrate_magnetometer()
{
  // ----- Locals
  Serial.println("Calib Magno, Wait..");
  int mag_x, mag_y, mag_z;
  int status_reg_2;                                               // ST2 status register

  int mag_x_min =  32767;                                         // Raw data extremes
  int mag_y_min =  32767;
  int mag_z_min =  32767;
  int mag_x_max = -32768;
  int mag_y_max = -32768;
  int mag_z_max = -32768;

  float chord_x,  chord_y,  chord_z;                              // Used for calculating scale factors
  float chord_average;

  // ----- Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000 ; counter ++)             // Run this code 16000 times
  {
    Loop_start = micros();                                        // Start loop timer

    // ----- Point to status register 1
    Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
    Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
    while (Wire.available() < 1);                                 // Wait for the data
    if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
    {
      // ----- Read data from each axis (LSB,MSB)
      Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
      while (Wire.available() < 7);                               // Wait for the data
      mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
      status_reg_2 = Wire.read();                                 // Read status and signal data read

      // ----- Validate data
      if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
      {
        // ----- Find max/min xyz values
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
    delay(4);                                                     // Time interval between magnetometer readings
  }

  // ----- Calculate hard-iron offsets
  Mag_x_offset = (mag_x_max + mag_x_min) / 2;                     // Get average magnetic bias in counts
  Mag_y_offset = (mag_y_max + mag_y_min) / 2;
  Mag_z_offset = (mag_z_max + mag_z_min) / 2;

  // ----- Calculate soft-iron scale factors
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;                 // Get average max chord length in counts
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;              // Calculate average chord length

  Mag_x_scale = chord_average / chord_x;                          // Calculate X scale factor
  Mag_y_scale = chord_average / chord_y;                          // Calculate Y scale factor
  Mag_z_scale = chord_average / chord_z;                          // Calculate Z scale factor

  // ----- Record magnetometer offsets
  /*
     When active this feature sends the magnetometer data
     to the Serial Monitor then halts the program.
  */
  if (Record_data == true)
  {
    // ----- Display data extremes
    Serial.print("XYZ Max/Min: ");
    Serial.print(mag_x_min); Serial.print("\t");
    Serial.print(mag_x_max); Serial.print("\t");
    Serial.print(mag_y_min); Serial.print("\t");
    Serial.print(mag_y_max); Serial.print("\t");
    Serial.print(mag_z_min); Serial.print("\t");
    Serial.println(mag_z_max);
    Serial.println("");

    // ----- Display hard-iron offsets
    Serial.print("Hard-iron: ");
    Serial.print(Mag_x_offset); Serial.print("\t");
    Serial.print(Mag_y_offset); Serial.print("\t");
    Serial.println(Mag_z_offset);
    Serial.println("");

    // ----- Display soft-iron scale factors
    Serial.print("Soft-iron: ");
    Serial.print(Mag_x_scale); Serial.print("\t");
    Serial.print(Mag_y_scale); Serial.print("\t");
    Serial.println(Mag_z_scale);
    Serial.println("");

    // ----- Display fuse ROM values
    Serial.print("ASA: ");
    Serial.print(ASAX); Serial.print("\t");
    Serial.print(ASAY); Serial.print("\t");
    Serial.println(ASAZ);

    // ----- Halt program
    while (true);                                       // Wheelspin ... program halt
  }
}

// -------------------------------
//  Read magnetometer
// -------------------------------
void read_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;

  // ----- Point to status register 1
  Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
  Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
  while (Wire.available() < 1);                                 // Wait for the data
  if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
  {
    // ----- Read data from each axis (LSB,MSB)
    Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
    while (Wire.available() < 7);                               // Wait for the data
    mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read

    // ----- Validate data
    if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
    {
      Mag_x = (mag_x - Mag_x_offset) * Mag_x_scale;
      Mag_y = (mag_y - Mag_y_offset) * Mag_y_scale;
      Mag_z = (mag_z - Mag_z_offset) * Mag_z_scale;
    }
  }
}
// -----------------------------------
//  Configure the gyro & accelerometer
// -----------------------------------
void config_gyro()
{
  //Serial.println("Configuring Gyro ..");
  // ----- Activate the MPU-6050
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x6B);                                     //Point to power management register
  Wire.write(0x00);                                     //Use internal 20MHz clock
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1C);                                     //Point to accelerometer configuration reg
  Wire.write(0x10);                                     //Select +/-8g full-scale
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1B);                                     //Point to gyroscope configuration
  Wire.write(0x08);                                     //Select 500dps full-scale
  Wire.endTransmission();                               //End the transmission
}

// -----------------------------------
//  Calibrate gyro
// -----------------------------------
void calibrate_gyro()
{

  //Serial.println("Calibrating Gyro ..");

  // ----- Calibrate gyro
  for (int counter = 0; counter < 64 ; counter ++)    //Run this code 64 times
  {
    Loop_start = micros();
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    Gyro_x_cal += Gyro_x;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    Gyro_y_cal += Gyro_y;                               //Add the gyro y-axis offset to the gyro_y_cal variable
    Gyro_z_cal += Gyro_z;                               //Add the gyro z-axis offset to the gyro_z_cal variable
    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  Gyro_x_cal = Gyro_x_cal >> 6;                                   //Divide the gyro_x_cal variable by 2048 to get the average offset
  Gyro_y_cal = Gyro_y_cal >> 6;                                   //Divide the gyro_y_cal variable by 2048 to get the average offset
  Gyro_z_cal = Gyro_z_cal >> 6;                                   //Divide the gyro_z_cal variable by 2048 to get the average offset


}


// -----------------------------------
//  Calibrate Accel .... Here You need Your MPU at Zero Level before Calibration, Otherwise You will Add an error.
//  Use Spirit Level to do that.
// -----------------------------------
void calibrate_Accel()
{

  //Serial.println("I am Assuiming You are Level ,Calibrating Accel ..");

  // ----- Calibrate Accel
  for (int counter = 0; counter < 64 ; counter ++)    //Run this code 1024 times
  {
    Loop_start = micros();
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    Accel_x_cal += Accel_x;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    Accel_y_cal += Accel_y;                               //Add the gyro y-axis offset to the gyro_y_cal variable

    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  Accel_x_cal = Accel_x_cal >> 6;                                   //Divide the gyro_x_cal variable by 64 to get the average offset
  Accel_y_cal = Accel_y_cal >> 6;                                   //Divide the gyro_y_cal variable by 64 to get the average offset


}



// --------------------
//  Read MPU 6050 data
// --------------------
void read_mpu_6050_data()
{
  /*
    Subroutine for reading the raw gyro and accelerometer data
  */

  // ----- Locals
  int     temperature;                                  // Needed when reading the MPU-6050 data ... not used

  // ----- Point to data
  Wire.beginTransmission(0x68);                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                     // Point to start of data
  Wire.endTransmission();                               // End the transmission

  // ----- Read the data
  Wire.requestFrom(0x68, 14);                           // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                        // Wait until all the bytes are received
  Accel_x = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_x variable
  Accel_y = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_y variable
  Accel_z = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_z variable
  temperature = Wire.read() << 8 | Wire.read();         // Combine MSB,LSB temperature variable
  Gyro_x = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_y = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_z = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
}


// The GO function Responsible for manouevring ans Start/Stop Engine

bool Go(float Brng, float head , float RelBrng, float Dist, byte fg) {


  // If We Arrive to our Destination Stop Engine and Signal That on Green Led
//  Serial.print("-- Distance : ");
//  Serial.print(Dist);
//  Serial.println(" m--");
//  Serial.print("-- RelBearing : ");
//  Serial.print(RelBrng);
//  Serial.println(" deg--");
  
  // See in Flags That there is no Problem with Communication and other thing

  if (fg & 0x1 || fg & 0x2 || fg & 0x3) {
    //There is a problem.
    // Stop Engine & Put the Rudder at 90°
    Engine(0);
    GouverSVO.write(90);
    digitalWrite (4, LOW);
    return false;
  }else if (Dist <= 5) {
    //if (target_done == 4) {Serial.println("!!In Postion!!");}
    Confirmation = Confirmation + 1; // the loop take about 5ms
    Engine(0);
    GouverSVO.write(90);
    digitalWrite (4, HIGH);
    // We get to our Target position
    
    if (Confirmation > 150){
    // Here We change the target to the next Postion
    
    // We have just 3 targets on our parcour , means 6 entries. check that we don't exceed this limit.
    // it is just a hack, we will change that with a reliable code in future.  
    if ((target_done + 2) <= 4 ){target_done = target_done + 2;}
    cible_actuelle[0] = parcour[target_done];
    cible_actuelle[1] = parcour[target_done + 1];
    Confirmation = 0;
    }
    return true;
    }else{
      digitalWrite (4, LOW);
    }
      
    
  if (abs(RelBrng) < 10) {
    
    GouverSVO.write(90);
    Engine(2);

  } else if ( head >= 0 && head <= 179) {

    Engine(1);
    //Where is My destination
    if (Bearing > 0 && Bearing < head) { // Destination is at PortSide.
      GouverSVO.write(60);
    } else if (Brng > head && Brng < (head + 180)) { // Destination at STBD.
      //Serial.println(RelBrng);
      GouverSVO.write(120);
    } else if (Brng > (head + 180) && Brng <= 359) {
      //Serial.println(RelBrng);
      GouverSVO.write(60);
    }

    // When Heading is between [180 , 359]
  } else if (head >= 180 && head <= 359) {
    Engine(1);
    //Where is My destination
    if (Brng > (head - 180) && Brng < head) { // Destination is at PortSide.
      //Serial.println(RelBrng);
      GouverSVO.write(60);

    } else if (Brng > head && Brng < 359) { // Destination at STBD.
      //Serial.println(RelBrng);
      GouverSVO.write(120);
    } else if (Brng > 0 && Brng <= (head - 180)) {
      //Serial.println(RelBrng);
      GouverSVO.write(120);
    }
  }
}

void Engine(int state) {
  if (state == 2) {
    
    
    analogWrite (ENB, 254);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
  }
  if (state == 1) {
    
    analogWrite (ENB, 150);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
  }
  if (state == 0) {  
      
    analogWrite (ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  
  }
