

#define ASCII_BIT 8
#define BAUD_RATE 10000

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
	for (i=0;i<8;i++)
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

void setup() {
  pinMode(9, OUTPUT); //Radio Tx0
  pinMode(11, OUTPUT); //Radio Tx1
  pinMode(7, OUTPUT); //Radio En
  digitalWrite(7,HIGH);
}

void loop() {
  rtty_txstring("Test Arduino\r\r\r");
}
