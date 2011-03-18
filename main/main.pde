/* 
Main program file. Will be used for testing until everything
works, then it'll be the centre of the program :)
*/


//Include GPS parser and set up an instance

#include <string.h>
#include <TinyGPS.h>
TinyGPS gps;

/*
GPS Connection, read, parse and export for UBLOX 5 enabled GPS Modules
*/

//char buffer[120]="";
//char pubxVerify[6]="$PUBX";
//char pubxString[21];
//int indices[25];
//int cont = 0;
//int bufinc=0;
//int ver=0;
char s[100];

#define ASCII_BIT 8
#define BAUD_RATE 10000


//Finally, the program can get going

void setup() {
  //Set up the Radio to do what we want
  pinMode(9, OUTPUT); //Radio Tx0
  pinMode(11, OUTPUT); //Radio Tx1
  pinMode(7, OUTPUT); //Radio En
  digitalWrite(7,HIGH);
  
         //Set up the GPS for doing what we want
	// Start up serial ports
	Serial1.begin(38400);
	Serial.begin(115200); // used for debug ouput
 
	delay(2000); // Give the GPS time to boot
 
	// Lower the baud rate to 9600 from 38.4k
	Serial.print("Setting uBlox port mode: ");
	uint8_t setPort[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E, 0x95};
	sendUBX(setPort, sizeof(setPort)/sizeof(uint8_t));
 
	// Switch baud rates on the software serial
	Serial.println("Switching to 9600b GPS serial");
	Serial1.begin(9600);
	delay(1000);
 
	// Set the navigation mode (Airborne, 1G)
	Serial.print("Setting uBlox nav mode: ");
	uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
	sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
	getUBX_ACK(setNav);
 
  //turn off all NMEA strings and request a UBLOX 00 string.
Serial1.println("$PUBX,40,GLL,0,0,0,0*5C");
Serial1.println("$PUBX,40,GGA,0,0,0,0*5A");
Serial1.println("$PUBX,40,GSA,0,0,0,0*4E");
Serial1.println("$PUBX,40,RMC,0,0,0,0*47");
Serial1.println("$PUBX,40,GSV,0,0,0,0*59");
Serial1.println("$PUBX,40,VTG,0,0,0,0*5E");
Serial1.println("$PUBX,00*33"); 
}
 
// Dump bytes to debug as they appear
void loop() {
  //buffer increment
  //check there is serial data available
  if (Serial1.available()) {
  int c=Serial1.read();
  if(gps.encode(c)){
  long lat, lon;
unsigned long fix_age, time, date, speed, course, pos;
unsigned long chars;
unsigned short sentences, failed_checksum;

// retrieves +/- lat/long in 100000ths of a degree
gps.get_position(&lat, &lon, &fix_age);

// time in hhmmsscc, date in ddmmyy
gps.get_datetime(&date, &time, &fix_age);

// returns speed in 100ths of a knot
speed = gps.speed();

pos = gps.sats();
 Serial.print(pos);
 
// course in 100ths of a degree
course = gps.course();

Serial.println(date);
Serial.println(time);


  }
   
  } //no serial data found: 
  else {
  //request serial data from gps
  Serial1.println("$PUBX,00*33"); 
  //set time before next poll
  delay(2000);
  }
}


/* This Function reads the data that's coming in the serial port from the GPS, 
verifies that it's accurate (by looking for $PUBX at the start, no checksumming yet),
sticks it in a few arrays, displays the values for debug and then puts them in a final
global array for RTTY transmission. Clear the buffer, wait to be called again 

void getPUBX() {
  //read serial data to buffer
       buffer[bufinc]=Serial1.read();
      
      //Uncomment below to see full string - prints to debug
      Serial.print(buffer[bufinc], BYTE);
     
      bufinc++;
      //when buffer is full, start parsing things
      if (bufinc==117) {
    cont=0;
    ver=0;
       for (int i=0;i<115;i++) {
         if (buffer[i] == pubxVerify[i]) {
           ver++;
         }
       
           if (ver==4) {
             
           for (int i=0;i<115;i++){
           if (buffer[i]==','){
             // check for the position of the  "," separator
             indices[cont]=i;
             cont++;
           }
           }
           //Purely for debug, spew out useful/interesting data
           Serial.println("");
           Serial.println("");
           Serial.println("---------------");
           for (int i=0;i<=20;i++) {
             switch(i){
             case 0 :Serial.print("Message Identifier: ");break;
             case 1 :Serial.print("Time in UTC (HhMmSs): ");break;
             case 2 :Serial.print("Latitude: ");break;
             case 3 :Serial.print("N/S Indicator: ");break;
             case 4 :Serial.print("Longitude: ");break;
             case 5 :Serial.print("E/W Indicator: ");break;
             case 6 :Serial.print("AltRef: ");break;
             case 7 :Serial.print("NavStat: ");break;
             case 8 :Serial.print("Hacc: ");break;
             case 9 :Serial.print("Vacc: ");break;
             case 10 :Serial.print("SOG ");break;
             case 11 :Serial.print("COG: ");break;
             case 12 :Serial.print("Vvel: ");break;
             case 13 :Serial.print("ageC: ");break;
             case 14 :Serial.print("HDOP: ");break;
             case 15 :Serial.print("VDOP: ");break;
             case 16 :Serial.print("TDop: ");break;
             case 17 :Serial.print("NoSats: ");break;
             case 18 :Serial.print("GLONASSats");break;
             case 19 :Serial.print("DR Used: ");break;
             case 20 :Serial.print("Checksum: ");break;             
           }
             
           for (int j=indices[i];j<(indices[i+1]-1);j++){
             Serial.print(buffer[j+1]); 
             pubxString[i] = buffer [j+1];
           }
           Serial.println("");
           }
           Serial.println("---------------");
          }
         }
         
          for (int i=0;i<117;i++){
           buffer[i]=' ';
           }
           bufinc=0;
         }
}
*/
