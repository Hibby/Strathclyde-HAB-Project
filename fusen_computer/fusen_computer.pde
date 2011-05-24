/*
* fusen_computer.pde
* Copyright (C) Dave Hibberd 2011 <dave.hibberd@gmail.com>
*
* This is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License
* as published by the Free Software Foundation; version 2.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses/>.
*/
	
#include <stdlib.h>
#include <OneWire.h>
#include <stdio.h>
#include <TinyGPS.h>
#include <stdint.h>
#include <util/crc16.h>
#include <string.h>
#include <floattostring.h>
#include <DallasTemperature.h>

TinyGPS gps;
OneWire ds(5);
DallasTemperature sensors(&ds);

/*
GPS Connection, read, parse and export for UBLOX 5 enabled GPS Modules
 */



DeviceAddress outside ={0x28, 0x36, 0x52, 0xEE, 0x02, 0x00, 0x00, 0x44};
DeviceAddress inside = {0x28, 0xC1, 0xB1, 0xE4, 0x02, 0x00, 0x00, 0x3C};

int counter=0;
char s[100];
char timestr[9];
char latstr[20];
char lonstr[20];
char intempstr[10];
char outtempstr[10];
unsigned long fix_age, time, date, speed, course, pos;
unsigned long chars;
unsigned short sentences, failed_checksum;
long lat, lon, alt;
float flat, flon, temp, intemp,outtemp;
#define ASCII_BIT 7
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

void rtty_txstring (char * string)
{
  /* Simple function to sent a char at a time to 
   	** rtty_txbyte function. 
   	** NB Each char is one byte (8 Bits)
   	*/
  char c;
  c = *string++;
  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}

void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	**
   	** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
   	*/
  int i;
  rtty_txbit (0); // Start bit
  // Send bits for for char LSB first	
  for (i=0;i<7;i++)
  {
    if (c & 1) rtty_txbit(1); 
    else rtty_txbit(0);	
    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    digitalWrite(9, HIGH);  
    digitalWrite(11, LOW);
  }
  else
  {
    // low
    digitalWrite(11, HIGH);
    digitalWrite(9, LOW);
  }
  //delayMicroseconds(20500); // 10000 = 100 BAUD 20150
  delayMicroseconds(BAUD_RATE); // 10000 = 100 BAUD 20150
}

//checksum send options

void make_string()
{
  char checksum[10];
  counter++;
  snprintf(s,sizeof(s),"$$FUSEN,%i,%s,%s,%s,%ld,%s;%s", counter, timestr, latstr, lonstr, alt, intempstr, outtempstr);

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


float printTemperature(DeviceAddress deviceAddress)
{
float tempC = sensors.getTempC(deviceAddress);

return tempC;
}

void setup() {
  // put your setup code here, to run once:
  //Set up the Radio to do what we want
  pinMode(9, OUTPUT); //Radio Tx0
  pinMode(11, OUTPUT); //Radio Tx1
  pinMode(7, OUTPUT); //Radio En
  digitalWrite(7,HIGH);
  pinMode(46,OUTPUT);
  pinMode(50,OUTPUT);
  digitalWrite(46,HIGH);
  digitalWrite(50,LOW);
  
  sensors.begin();
  
  sensors.setResolution(inside,10);
  sensors.setResolution(outside,10);

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

       //check there is serial data available
  if (Serial1.available()) {
    int c=Serial1.read();
    if(gps.encode(c)){



      // retrieves +/- lat/long in 100000ths of a degree
      gps.get_position(&lat, &lon, &fix_age);

      // time in hhmmsscc, date in ddmmyy
      gps.get_datetime(&date, &time, &fix_age);

      sensors.requestTemperatures();
  
       intemp = printTemperature(inside);
       outtemp = printTemperature(outside);

  
   floatToString(intempstr,intemp,2);
   floatToString(outtempstr,outtemp,2);
        
      
      
      flat = lat;
      flon = lon;
      flat = flat/100000;
      flon = flon/100000;
      floatToString(latstr,flat,7);
      floatToString(lonstr,flon,7);
      alt = gps.altitude()/100;
 
      
     //final functions to make everything come together
      Serial.println(s);
      timeSort();
      make_string();
      
      rtty_txstring(s);
      delay(5000);
    }

  } //no serial data found: 
  else {
  //reset condition, no cutoff
    //request serial data from gps
    Serial1.println("$PUBX,00*33"); 
    
    delay(2000);
    
    /*rtty_txstring("$$FUSEN,time, lat, lon, alt, ");
    rtty_txstring(intempstr);
    rtty_txstring(";");
    rtty_txstring(outtempstr);
    rtty_txstring(" *checksum\n");
    */
    //set time before next poll
    
  }
}
