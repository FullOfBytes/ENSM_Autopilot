
#include <TinyGPS++.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>



float Bearing  = 0;
float Distance = 0;
float RelativeBearing = 0;
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
const byte address[][6] = {"00001", "00002"}; // Address For nRF24

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

typedef struct
{
  char cmd = '0'; // 1 meaning Transmitter mode for PC-Station
 
} Com_Struct;
Com_Struct Command;



float babor_Lat = 0;
float babor_Lng = 0;
float cible[2] = {36.649881562818, 2.69535783534598}; // Target Coordinates Lat, Long

// GPS
TinyGPSPlus gps;


//-------------------------------------------
// Initialization of diffrent Objects

// nRF24
RF24 radio(7, 8);   // nRF24L01 (CE, CSN)
// 13 SCK
// 12 MISO
// 11 MOSI


void setup() {


  // Serial init ---------------------------
  Serial.begin(115200);

  // nRF24 Init ----------------------------

  radio.begin();
  radio.openWritingPipe(address[0]);  // Address to use to transmit
  radio.openReadingPipe(1, address[1]); // Addresse to use for recieving.
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();


}

void loop() {
  if (bitRead(Infos.flags, 0)) {
   // Serial.println("Babor Lost Serial Connection to phone");
    }
//  if (bitRead(Infos.flags, 1)) {
//    Serial.println("");
//    }
  
  if (Serial.available()) {
    Command.cmd = Serial.read();
    
  }


  switch (Command.cmd ) {
    case '0':

      //Serial.println(F("Receiver Mode..."));
      

      // Check whether there is data to be received
      while (!radio.available())
      { //Serial.println(F("No Data Available on Radio"));
        }

      radio.read(&Infos, sizeof(Infos)); // Read the whole data and store it into the 'Infos' structure

      babor_Lat = Infos.Loc_Lat;
      babor_Lng = Infos.Loc_Lng;

      // Calculate Bearing From GPS Coordinates
      Bearing  = gps.courseTo(babor_Lat, babor_Lng, cible[0], cible[1]);
      Distance = gps.distanceBetween(babor_Lat, babor_Lng, cible[0], cible[1]);

     //  RelativeBearing = Bearing - Infos.heading;

//      Serial.println("----------");
//      Serial.print(" My Latitude: ");
//      Serial.print(Infos.Loc_Lat, 6);
//      Serial.print(" My Longitude: ");
//      Serial.println(Infos.Loc_Lng, 6);
//
//      Serial.println("----------");
//        Serial.print(F("Relative Bearing:  "));
//        Serial.println(Infos.RelativeBearing);
//        Serial.print(F("GPS Speed:  "));
//        Serial.println(Infos.speed);
//      Serial.println(F(" deg"));
//      Serial.println("----------");
//
//        //Serial.print("Roll : ");
//        Serial.print(Infos.roll);  
//        Serial.print("\t");
//       // Serial.print("Pitch : ");
//        Serial.print(Infos.pitch);  
//        Serial.print("\t");
//       // Serial.print("Yaw : ");
//        Serial.println(Infos.yaw);  


      
       Serial.print(Infos.roll);  
       Serial.print("/");
       Serial.print(Infos.pitch);  
       Serial.print("/");
       Serial.println(Infos.yaw);
       Serial.println(Infos.RelativeBearing);


      break;


      
    case '1': 
      // Tell Babor that you will switch to Transmitter Mode.
     
      Serial.println(F("Transmitter Mode..."));
      delay(5);
      radio.stopListening();

      while(!Serial.available()){Serial.println(F("Type a Command : "));}
     
      Command.cmd = Serial.parseInt();
      radio.write(&Command, sizeof(Command));
     
      break;
  }

   delay(20); // For not overwhelming Proccessing 
}
