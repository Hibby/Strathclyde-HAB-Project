#include <OneWire.h>
#include <stdlib.h>
#include <stdio.h>
#include <TinyGPS.h>
#include <stdint.h>
#include <util/crc16.h>
#include <string.h>
#include <floattostring.h>
#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))

#define SPEED  (50)
#define DOTLEN  (1200/SPEED)
#define DASHLEN  (3*(1200/SPEED))

int LEDpin = 12 ;

OneWire ds(8);
TinyGPS gps;

/*
GPS Connection, read, parse and export for UBLOX 5 enabled GPS Modules
 */

int counter=0;
char s[100];
char timestr[9];
char latstr[20];
char lonstr[20];
char tempstr[10];
unsigned long fix_age, time, date, speed, course, pos;
unsigned long chars;
unsigned short sentences, failed_checksum;
long lat, lon, alt;
float flat, flon, temp;
#define ASCII_BIT 8
#define BAUD_RATE 10000


// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial1.print(MSG[i], BYTE);
    Serial.print(MSG[i], HEX);
  }
  Serial.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}

//Next up rtty

// Start of Morse Code Generator //
struct t_mtab { char a, pat; } ;

struct t_mtab morsetab[] = {
  	{'.', 106},
	{',', 115},
	{'?', 76},
	{'/', 41},
	{'A', 6},
	{'B', 17},
	{'C', 21},
	{'D', 9},
	{'E', 2},
	{'F', 20},
	{'G', 11},
	{'H', 16},
	{'I', 4},
	{'J', 30},
	{'K', 13},
	{'L', 18},
	{'M', 7},
	{'N', 5},
	{'O', 15},
	{'P', 22},
	{'Q', 27},
	{'R', 10},
	{'S', 8},
	{'T', 3},
	{'U', 12},
	{'V', 24},
	{'W', 14},
	{'X', 25},
	{'Y', 29},
	{'Z', 19},
	{'1', 62},
	{'2', 60},
	{'3', 56},
	{'4', 48},
	{'5', 32},
	{'6', 33},
	{'7', 35},
	{'8', 39},
	{'9', 47},
	{'0', 63}
} ;


void
dash()
{
  digitalWrite(LEDpin, LOW) ;
  delay(DASHLEN);
  digitalWrite(LEDpin, HIGH) ;
  delay(DOTLEN) ;
}

void
dit()
{
  digitalWrite(LEDpin, LOW) ;
  delay(DOTLEN);
  digitalWrite(LEDpin, HIGH) ;
  delay(DOTLEN);
}

void
send(char a)
{
  int i ;
  if (a == ' ') {
    Serial.print(a) ;
    delay(7*DOTLEN) ;
    return ;
  }
  for (i=0; i<N_MORSE; i++) {
    if (morsetab[i].a == a) {
      unsigned char p = morsetab[i].pat ;
     Serial.print(morsetab[i].a) ; //- Killed echo back to serial

      while (p != 1) {
          if (p & 1)
            dash() ;
          else
            dit() ;
          p = p / 2 ;
      }
      delay(2*DOTLEN) ;
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
  //Serial.print("?") ;
}

void
sendmsg(char *str)
{
  while (*str)
    send(*str++) ;
  //Serial.println(""); //-- Killed LF back to serial
}



//checksum send options

void make_string()
{
  char checksum[10];
  counter++;
  snprintf(s,sizeof(s),"??EASYJET1,%i,%s,%s,%s,%ld", counter, timestr, latstr, lonstr, alt);

  snprintf(checksum, sizeof(checksum), "*%04X\n", gps_CRC16_checksum(s));
  // or 	snprintf(checksum, sizeof(checksum), "*%02X\n", gps_xor_checksum(s));

  // It would be much more efficient to use the return value of snprintf here, rather than strlen

  if (strlen(s) > sizeof(s) - 4 - 1)
  {
    // Don't overflow the buffer. You should have made it bigger.
    return;
  }

  // Also copy checksum's terminating \0 (hence the +1).
  memcpy(s + strlen(s), checksum, strlen(checksum) + 1);
}

uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}
//Converts time to character array, Adds colons 
void timeSort() {
      sprintf(timestr,"%lu",time);
      timestr[6] = timestr[4];
      timestr[7] = timestr[5];
      timestr[5] = ':';
      timestr[4] = timestr[3];
      timestr[3] = timestr[2];
      timestr[2] = ':';
}


void setup() {
  // put your setup code here, to run once:
  //Set up the Radio to do what we want
  //pinMode(9, OUTPUT); //Radio Tx0
  //pinMode(11, OUTPUT); //Radio Tx1
  //pinMode(7, OUTPUT); //Radio En
    pinMode(LEDpin, OUTPUT) ;
 // digitalWrite(7,HIGH);
  pinMode(46,OUTPUT);
  pinMode(50,OUTPUT);
  digitalWrite(46,HIGH);
  digitalWrite(50,LOW);

  //Set up the GPS for doing what we want
  // Start up serial ports
  Serial1.begin(38400);
  Serial.begin(115200); // used for debug ouput

  delay(2000); // Give the GPS time to boot

  // Lower the baud rate to 9600 from 38.4k
  Serial.print("Setting uBlox port mode: ");
  uint8_t setPort[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E, 0x95  };
  sendUBX(setPort, sizeof(setPort)/sizeof(uint8_t));

  // Switch baud rates on the software serial
  Serial.println("Switching to 9600b GPS serial");
  Serial1.begin(9600);
  delay(1000);

  // Set the navigation mode (Airborne, 1G)
  Serial.print("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC  };
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

void loop() {
  // put your main code here, to run repeatedly: 
  //counter increment
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
    return;
  } 
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad


  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();

  }
      
  temp = ( (data[1] << 8) + data[0] )*0.0625;
  Serial.print(temp);
  
        floatToString(tempstr,temp,2);
      Serial.print(tempstr);
  //check there is serial data available
  if (Serial1.available()) {
    int c=Serial1.read();
    if(gps.encode(c)){

      // retrieves +/- lat/long in 100000ths of a degree
      gps.get_position(&lat, &lon, &fix_age);

      // time in hhmmsscc, date in ddmmyy
      gps.get_datetime(&date, &time, &fix_age);

      // returns speed in 100ths of a knot
      speed = gps.speed();

      pos = gps.sats();

      // course in 100ths of a degree
      course = gps.course();
      flat = lat;
      flon = lon;
      flat = flat/100000;
      flon = flon/100000;
      floatToString(latstr,flat,7);
      floatToString(lonstr,flon,7);

      alt = gps.altitude()/100;
      if (alt > 24000) {
        //cutoff
        digitalWrite(50,HIGH);
        digitalWrite(46,LOW);
      } else {
        //reset condition, no cutoff
        digitalWrite(46,HIGH);
        digitalWrite(50,LOW);
      }
      
     //final functions to make everything come together
      Serial.println(s);
      timeSort();
      make_string();
      
      sendmsg(s);
      delay(60000);
    }

  } //no serial data found: 
  else {
  //reset condition, no cutoff
  digitalWrite(46,HIGH);
  digitalWrite(50,LOW);
    //request serial data from gps
    Serial1.println("$PUBX,00*33"); 
    sendmsg("TEMPERATURE: ");
    sendmsg(tempstr);
    Serial.println(tempstr);
    //set time before next poll
    delay(2000);
  }
}
