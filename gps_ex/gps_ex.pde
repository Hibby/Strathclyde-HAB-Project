// This code gives an example of configuring an FSA03 connected to a software serial port on an Arduino
 
char buffer[120]="";
char pubxVerify[6]="$PUBX";
int indices[25];
int cont = 0;
int bufinc=0;
int ver=0;

void setup() {
 
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
Serial1.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
Serial1.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
Serial1.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
Serial1.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
Serial1.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
Serial1.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
  Serial1.println("$PUBX,00*33"); 
 }
 
// Dump bytes to debug as they appear
void loop() {
  //buffer increment
  //check there is serial data available
  if (Serial1.available()) {

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
 
  //no serial data found: 
  else {
  //request serial data from gps
  Serial1.println("$PUBX,00*33"); 
  //set time before next poll
  delay(2000);
  }
}
  
 
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
			} else {
				ackByteID = 0;	// Reset and look again, invalid order
			}
 
		}
	}
}

