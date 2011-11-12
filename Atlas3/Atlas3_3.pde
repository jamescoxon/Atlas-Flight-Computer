#include <TinyGPS.h>
#include <OneWire.h>
#include <stdio.h>
#include <util/crc16.h>
#include <PID_Beta6.h>

TinyGPS gps;
OneWire ds(12); // DS18x20 Temperature chip i/o One-wire

//Tempsensor variables
byte address0[8] = {0x28, 0x25, 0x55, 0xC2, 0x2, 0x0, 0x0, 0x75}; // Radio DS18B20 28 25 55 C2 2 0 0 75
byte address1[8] = {0x28, 0x6F, 0x8A, 0xC2, 0x2, 0x0, 0x0, 0xFE}; // Internal DS18B20 28 6F 8A C2 2 0 0 FE
byte address2[8] = {0x28, 0xE8, 0x89, 0xC2, 0x2, 0x0, 0x0, 0xDF}; // External DS18B20 GPS Sensor 28 E8 89 C2 2 0 0 DF
int temp0 = 0, temp1 = 0, temp2 = 0;
int photo = 0;

//PID Controller
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,10,5,0);
int iOutput = 0;

int count = 1, nightloop = 0;
byte navmode = 99;
float flat, flon;
unsigned long date, time, chars, age;

int hour = 0 , minute = 0 , second = 0, oldsecond = 0;
char latbuf[12] = "0", lonbuf[12] = "0", altbuf[12] = "0";
long int ialt = 123;
int numbersats = 99;

int battV;

// ------------------------
// RTTY Functions - from RJHARRISON's AVR Code
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
                    digitalWrite(4, HIGH);  
                    digitalWrite(5, LOW);
                    digitalWrite(13,HIGH);
		}
		else
		{
		  // low
                    digitalWrite(5, HIGH);
                    digitalWrite(4, LOW);
                    digitalWrite(13, LOW);
		}
		//delayMicroseconds(20500); // 10000 = 100 BAUD 20150
                //delayMicroseconds(20000); // 10000 = 100 BAUD 20150
                delayMicroseconds(19500); // 10000 = 100 BAUD 20150

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
  
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.print(MSG[i], BYTE);
  }
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
	uint8_t b;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
        Serial.flush();
	unsigned long startTime = millis();
 
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
                                navmode = 1;
				return true;
		}
 
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) { 
                        navmode = 0;
			return false;
		}
 
		// Make sure data is available to read
		if (Serial.available()) {
			b = Serial.read();

			// Check that bytes arrive in sequence as per expected ACK packet
			if (b == ackPacket[ackByteID]) { 
				ackByteID++;
                                //Serial.print(ackPacket[ackByteID], HEX);
                                //Serial.print(" ");
			} else {
				ackByteID = 0;	// Reset and look again, invalid order
			}
 
		}
	}
}

//Function to poll the NAV5 status of a Ublox GPS module (5/6)
//Sends a UBX command (requires the function sendUBX()) and waits 3 seconds
// for a reply from the module. It then isolates the byte which contains 
// the information regarding the NAV5 mode,
// 0 = Pedestrian mode (default, will not work above 12km)
// 6 = Airborne 1G (works up to 50km altitude)
//Adapted by jcoxon from getUBX_ACK() from the example code on UKHAS wiki
// http://wiki.ukhas.org.uk/guides:falcom_fsa03
boolean checkNAV(){
  uint8_t b, bytePos = 0;
  uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 }; //Poll NAV5 status
  
  Serial.flush();
  unsigned long startTime = millis();
  sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));
  
  while (1) {
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
      
      if(bytePos == 8){
        navmode = b;
        return true;
      }
                        
      bytePos++;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      navmode = 0;
      return false;
    }
  }
}

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  
  delay(3000); // Wait for the GPS to process all the previous commands
  
 // Check and set the navigation mode (Airborne, 1G)
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  
  getUBX_ACK(setNav);
  
}

// gets temperature data from onewire sensor network, need to supply byte address, it'll check to see what type of sensor and convert appropriately
int getTempdata(byte sensorAddress[8]) {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole;
  byte data[12], i, present = 0;
  
  ds.reset();
  ds.select(sensorAddress);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(sensorAddress);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
 LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  
  if (sensorAddress[0] == 0x10) {
    Tc_100 = TReading * 50;    // multiply by (100 * 0.0625) or 6.25
  }
  else { 
    Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
  }
  
  
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions

  if (SignBit) // If its negative
  {
     Whole = Whole * -1;
  }
  return Whole;
}

void lightsensor() {
  int photocellPin = 0; // the cell and 10K pulldown are connected to a0
  //Photocell/ Photodiode External Light Reading
  photo = analogRead(photocellPin);
  photo = photo / 10;
}

void radiotemp() {
        //Measure temperature sensor and adjust heater.
//        if(battV > 350){
          temp0 = getTempdata(address0);
          if((temp0 < 50) && (temp0 > -50))
          {
            Input = double(temp0);
            myPID.Compute();
            analogWrite(10,Output);
          }
   //     }
 //       else{
 //         iOutput = -1;
 //       }
}

void setup()
{
  pinMode(5, OUTPUT); //Radio Tx0
  pinMode(4, OUTPUT); //Radio Tx1
  pinMode(6, OUTPUT); //Radio En
  digitalWrite(6, HIGH);
  Serial.begin(9600);
  
  delay(5000); // We have to wait for a bit for the GPS to boot otherwise the commands get missed
  
  setupGPS();
  
  Setpoint = 2;
  myPID.SetMode(AUTO);
  
  analogReference(DEFAULT);

}

void loop() { 
    char superbuffer [120];
    char checksum [10];
    int n;
    
    if((count % 10) == 0) {
     digitalWrite(6, LOW);
     checkNAV();
     delay(1000);
     if(navmode != 6){
       setupGPS();
       delay(1000);
     }
     checkNAV();
     delay(1000);
     digitalWrite(6, HIGH);
   }
   
    Serial.println("$PUBX,00*33"); //Poll GPS
    
    while (Serial.available())
    {
      int c = Serial.read();
      if (gps.encode(c))
      {
        //Get Data from GPS library
        //Get Time and split it
        gps.get_datetime(&date, &time, &age);
        hour = (time / 1000000);
        minute = ((time - (hour * 1000000)) / 10000);
        second = ((time - ((hour * 1000000) + (minute * 10000))));
        second = second / 100;
      
      //Get Position
      gps.f_get_position(&flat, &flon);
  
      //convert float to string
      dtostrf(flat, 7, 4, latbuf);
      dtostrf(flon, 7, 4, lonbuf);
      
      //just check that we are putting a space at the front of lonbuf
      if(lonbuf[0] == ' ')
      {
        lonbuf[0] = '+';
      }
      
      // +/- altitude in meters
      ialt = (gps.altitude() / 100);   
      itoa(ialt, altbuf, 10);
    }
    }
    
    battV = analogRead(1);
    temp0 = getTempdata(address0);
    temp1 = getTempdata(address1);
    temp2 = getTempdata(address2);
    lightsensor();
    iOutput = int(Output);
    numbersats = gps.sats();
    
    radiotemp();
    n=sprintf (superbuffer, "$$ATLAS,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d,%d,%d,%d,%d,%d,%d", count, hour, minute, second, latbuf, lonbuf, ialt, numbersats, navmode, battV, temp0, temp1, temp2, iOutput, photo);
    if (n > -1){
      n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
      rtty_txstring(superbuffer);
      
    }
    count++;

}
