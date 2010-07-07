#include "TinyGPS_2.h"
TinyGPS gps;

#include <OneWire.h> 
OneWire ds(9); // DS18x20 Temperature chip i/o One-wire

#define ASCII_BIT 8
#define BAUD_RATE 20150     // 10000 = 100 BAUD 20150
#define SERIAL_SPEED 9600

//GPIO pin definitions
int en = 8, tx0 = 3, tx1 = 4, led = 13, pump_relay = 12;

int count = 0, nogps_count = 0, gpsairborne = 0;
float flat, flon, falt;
unsigned long fix_age, date, time, chars, age;
unsigned short sentences, failed_checksum, failed;
uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  

int hour, minute, second, lat_deg, lat_min, cutdown = 0, numbersats;
char latbuf[12], lonbuf[12], ascentratebuf[12], ialtbuf[10];
long int ialt = 123;

//Ascent rate variables
long int currenttime, ARtime = 0, Floatstart, float_time = 0, total_float_time;
float ascentrate;
int Float_rollover_time = 0, atfloat = 0, last_alt, descent_detection = 10, too_low = 0;

//Tempsensor variables
byte address1[8] = {0x28, 0x2D, 0x28, 0x2E, 0x2, 0x0, 0x0, 0xE}; // Internal DS18B20 Temp Sensor
byte address2[8] = {0x28, 0x5D, 0x26, 0x2E, 0x2, 0x0, 0x0, 0x34}; // External DS18B20 Temp Sensor
int temp0 = 0, temp1 = 0; 

//Photocell Data
int light0 = 0;

//Ballast variables
int totalPumpMls = 500, ballastmode = 0, pumpCount = 0, dumpMls = 100, thisDumpMls = 0, dje_backup = 0, nogps_ballast = 0;
float pumpMls;
long int motor_turn_time = 0;

char superbuffer [120];
char checksum [5];
int n, val = 1;
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
	for (i=0;i<ASCII_BIT;i++)
	{
		if (c & 1) rtty_txbit(1); 
			else rtty_txbit(0);	
		c = c >> 1;
	}
	rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                    digitalWrite(tx1, HIGH);
                    digitalWrite(tx0, LOW);
                    digitalWrite(led, LOW); //LED  
		}
		else
		{
		  // low
                    digitalWrite(tx0, HIGH);
                    digitalWrite(tx1, LOW);
                    digitalWrite(led, HIGH); //LED
		}
		delayMicroseconds(BAUD_RATE); 
}

char * floatToString(char * outstr, double val, byte precision, byte widthp){
  char temp[16];
  byte i;
  // compute the rounding factor and fractional multiplier
  double roundingFactor = 0.5;
  unsigned long mult = 1;
  for (i = 0; i < precision; i++)
  {
    roundingFactor /= 10.0;
    mult *= 10;
  }
  
  temp[0]='\0';
  outstr[0]='\0';

  if(val < 0.0){
    strcpy(outstr,"-\0");
    val = -val;
  }

  val += roundingFactor;

  strcat(outstr, itoa(int(val),temp,10));  //prints the int part
  if( precision > 0) {
    strcat(outstr, ".\0"); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
      mult *=10;

    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;

    while(frac1 /= 10)
      padding--;

    while(padding--)
      strcat(outstr,"0\0");

    strcat(outstr,itoa(frac,temp,10));
  }

  // generate space padding
  if ((widthp != 0)&&(widthp >= strlen(outstr))){
    byte J=0;
    J = widthp - strlen(outstr);
    
    for (i=0; i< J; i++) {
      temp[i] = ' ';
    }

    temp[i++] = '\0';
    strcat(temp,outstr);
    strcpy(outstr,temp);
  }
  
  return outstr;
} 


unsigned int gps_checksum (char * string)
{	
	unsigned int i;
	unsigned int XOR;
	unsigned int c;
	// Calculate checksum ignoring any $'s in the string
	for (XOR = 0, i = 0; i < strlen(string); i++)
	{
		c = (unsigned char)string[i];
		if (c != '$') XOR ^= c;
	}
	return XOR;
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

//Ascent rate calculation
void calc_ascentrate() {
  int diff_sec, diff_alt;
  if(ARtime == 0) {
    ARtime = (((long) hour * 3600) + (minute * 60) + second);
    } 
    else {
	if(currenttime > ARtime) {
	  diff_sec = currenttime - ARtime;
	  diff_alt = ialt - last_alt;
	  Serial.print("Diff sec = ");
          Serial.print(diff_sec);
          Serial.print(", Diff alt = ");
          Serial.println(diff_alt);
	  if (diff_sec > 60) {
		ascentrate = (float) diff_alt / diff_sec;
		Serial.print("Ascentrate: ");
                Serial.println(ascentrate);
		ARtime = currenttime;
		last_alt = ialt;
	  }
	}
	if(currenttime < ARtime) {
	  ARtime = currenttime;
	  last_alt = ialt;
	}
	  Serial.print("Time: ");
          Serial.println(currenttime);
  }
}

void floatdetection()
{
	//Calculate if we have achieved float, monitor ascentrate if between +1 and -1 start float time if it strays out of +1 or -1 for more then 5 cycles - float has stopped
	//Detect float
	if (ascentrate < 1 && ascentrate > -1 && ialt > 15000) {
	Serial.print("Floating... ");
	//Now that we are at float we need to keep track of how long
	  if (currenttime >= Floatstart) {
		//Start Timer or Continue Timer
		if(float_time == 0) {
			Floatstart = currenttime;
			atfloat = 10;
			float_time = 1 ;
			Serial.print("Start float timer ");
                        Serial.println(atfloat);
			}
		else {
			atfloat = 10;
			float_time = (long) currenttime - Floatstart;
			total_float_time = (long)float_time + Float_rollover_time;
			Serial.print("Float Time: ");
                        Serial.print(total_float_time);
                        Serial.print(" ");
                        Serial.println(atfloat);
		}
	}
	  else if (currenttime < Floatstart) {
		atfloat = 10;
		//We must have rolled over 24hrs
		Float_rollover_time = float_time;
		Serial.print("Rollover: ");
                Serial.println(Float_rollover_time);
		Floatstart = currenttime;
	}
	}
	// If out of float - detect
	else {
		if (atfloat > 0) {
			//reduce float counter by 1
			atfloat = atfloat - 1;
			//in case this is just a blip continue float counter until we run out
			float_time = currenttime - Floatstart;
			total_float_time = float_time + Float_rollover_time;
			Serial.print("Float Time: ");
                        Serial.print(total_float_time);
                        Serial.print(" ");
                        Serial.println(atfloat);
			}
		else {
		  float_time = 0;
                  total_float_time = 0;
		}
		Serial.print("No Float: ");
                Serial.println(atfloat);
	}
}

void startBallast() {
  //Attach interrupt and start counting
  attachInterrupt(0, counter, RISING);
  //Start dumping ballast turn on pump
  digitalWrite(pump_relay, HIGH);
}

void stopBallast() {
  //Stop pump
  digitalWrite(pump_relay, LOW);
  //Disconnect interrupt
  detachInterrupt(0);
  ballastmode = 0;
  total_float_time = 0;
  float_time = 0;
  thisDumpMls = 0;
  too_low = 0;
}

void lightsensor() {
  int photocellPin = 5; // the cell and 10K pulldown are connected to a0
  //Photocell/ Photodiode External Light Reading
  light0 = analogRead(photocellPin);
  light0 = light0 / 10;
}
 
void counter()
{
  pumpCount++;
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.print(MSG[i], BYTE);
  }
  Serial.println();
}

// Get the current NAV5 mode
int getUBXNAV5() {

	uint8_t b;
	uint8_t byteID = 0;
	int startTime = millis();

	// Poll/query message
	uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 };
	
	// First few bytes of the poll response
	uint8_t response[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF};
	
	// Interrogate Mr GPS...
	sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));
	
	// Process his response...
	while (1) {
		
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) { 
			return -1;
		}

		// Make sure data is available to read
		if (Serial.available()) {
			b = Serial.read();

			// 8th byte is the nav mode
			if (byteID == 8) {
				return b;
			} 
			// Make sure the response matches the expected preamble
			else if (b == response[byteID]) { 
				byteID++;
			} else {
				byteID = 0;	// Reset and look again, invalid order
			}
			 
		}
	}

}

void setup()
{
  pinMode(en, OUTPUT); //EN
  pinMode(tx0, OUTPUT); //Tx
  pinMode(tx1, OUTPUT); //Tx
  pinMode(led, OUTPUT); //LED
  pinMode(pump_relay, OUTPUT); //Pump Relay
  Serial.begin(SERIAL_SPEED);
  digitalWrite(en, HIGH);
  
    //Turning off all GPS NMEA strings apart from GPGGA on the uBlox modules
  Serial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  Serial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  Serial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  Serial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  Serial.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  delay(2000);
  // Set the navigation mode (Airborne, 1G)
  rtty_txstring("Setting uBlox nav mode: ");
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  delay(100);
  if(getUBXNAV5() != 6) {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
      rtty_txstring("Resending UBX Command");  
  }
  else {
    rtty_txstring("Ublox in Airborne Mode"); 
  }
  

  //
  digitalWrite(led, HIGH);

  digitalWrite(pump_relay, HIGH);
  delay(2000);
  Serial.println("Starting Up...");
  digitalWrite(pump_relay, LOW);
  digitalWrite(led, LOW);
}

void loop()
{
  while (Serial.available())
  {
    int c = Serial.read();
    if (gps.encode(c))
    {
      // retrieves +/- lat/long in 100000ths of a degree
      if (fix_age == TinyGPS::GPS_INVALID_AGE) {
          //Serial.println("No fix detected");
      }
      else {
        //Get Data from GPS library
        //Get Time and split it
        gps.get_datetime(&date, &time, &age);
        hour = (time / 1000000);
        minute = ((time - (hour * 1000000)) / 10000);
        second = ((time - ((hour * 1000000) + (minute * 10000))));
        second = second / 100;
        //Get Position
        gps.f_get_position(&flat, &flon);
        falt = gps.f_altitude(); // +/- altitude in meters
        ialt = long(falt);
        
        //Reset nogps_ballast timer - just so that we don't stop ballast if by a mistake drop gps for 1 or 2 loops
        nogps_ballast = 0;
        
        //Calc Time
        currenttime =  (((long) hour * 3600) + (minute * 60) + second);
        
        //Ascent rate calculation
        calc_ascentrate();
        
        //Calculate if we have achieved float, monitor ascentrate if between +1 and -1 start float time if it strays out of +1 or -1 for more then 5 cycles - float has stopped
        floatdetection();

       // There are 4 situations that we could drop ballast
       // Mode 1 = if total_float_time is more then 600 (10mins) and we've still got ballast left
       // Mode 2 = Emergency drop if we are descending
       // Mode 3 = DJE Drop - because the payload is drifting too far
       // Mode 4 = if the ascent rate is too slow

        
        // Check to see that we aren't all ready dropping ballast and make sure that enough time has passed  
        if(ballastmode == 0 && total_float_time > 600 && totalPumpMls > 0){
          if(totalPumpMls <= 400 && ialt > 30000) {
            ballastmode = 0;
          }
          else {
           ballastmode = 1;
           startBallast();
          }
        }
 
         //This is the emergency ballast dump - If we find that we are decending then it does a crazy ballast dump slightly to test the tanks but also it 'might' rescue the payload
        if(ballastmode == 0 && ascentrate < -5 && totalPumpMls > 0) {
          if(descent_detection <= 1) {
            descent_detection = 10;
            ballastmode = 2;
            startBallast();
          }
          else {
            descent_detection = descent_detection - 1;
          }
        }
        
        // This is the backup if we are getting too far
        if(ballastmode == 0 && flat < 50.0 && dje_backup == 0) {
          ballastmode = 3;
          dje_backup = 1;
          startBallast();
        }
        
        //Backup dump if ascentrate is too slow and we are below 5000m
        if(ballastmode == 0 && ialt > 500 && ialt < 5000 && ascentrate < 1.5) {
         if(too_low > 10) {
          ballastmode = 4; 
          startBallast();
         }
         else {
           too_low++;
         }
        }
        else {
          too_low = 0;
        }
        
        if(ballastmode > 0) {
          //We are dumping ballast, how much have we dumped? Convert revolutions to mls and zero counter
            //Convert pumpCounter;
            pumpMls = (float) pumpCount * 0.26; //
            //Total up ballast this dump
            thisDumpMls = thisDumpMls + (int) pumpMls;
            Serial.print(pumpCount); Serial.print(",");Serial.print(pumpMls); Serial.print(","); Serial.println(thisDumpMls);
            pumpCount = 0; //Zeroing counter
          //Should we stop dumping ballast?
          if(ballastmode == 4) {
              dumpMls = 25;
          }
          else {
            dumpMls = 100;
          }
          
          if(thisDumpMls >= dumpMls) {
            //Stop pump
            stopBallast();
          }
          //Whats the total ballast that has been dumped?
          totalPumpMls = totalPumpMls - (int) pumpMls;
        }
        

        
        floatToString(ascentratebuf, ascentrate, 1, 0);
        floatToString(latbuf, flat, 4, 0);
        floatToString(lonbuf, flon, 4, 0);
        delay(100);
        temp0 = getTempdata(address1);
        delay(100);
        temp1 = getTempdata(address2);
        delay(100);
        lightsensor();
        numbersats = gps.num_sats();
        
        n=sprintf (superbuffer, "$$ATLAS,%d,%d:%d:%d,%s,%s,%ld,%d,%s;%d;%ld;%d;%d;%d;%d;%d", count, hour, minute, second, latbuf, lonbuf, ialt, numbersats, ascentratebuf, atfloat,float_time,temp0,temp1,light0,totalPumpMls,ballastmode);
        if (n > -1){
          n = sprintf (checksum, "*%02X\n",gps_checksum(superbuffer));
          n = sprintf (superbuffer, "%s%s", superbuffer, checksum);
          rtty_txstring(superbuffer);
          Serial.println(superbuffer);
        }
        count++;
        delay(100);
      }
    }
  }
  //No GPS Input
  nogps_count++;
  delay(10);
  // Turns LED on if we have got more then 1 sat - good way to make sure that everything is working without attaching to the serial port
  if (numbersats > 0 && numbersats != 99) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
  if (nogps_count > 1000) {
    if(gpsairborne >=20) {
        if(getUBXNAV5() != 6) {
          rtty_txstring("Resending UBX Command"); 
          sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
          gpsairborne = 0;
        }
        else {
          rtty_txstring("Ublox in Airborne Mode"); 
          gpsairborne = 0;
        }
    }
    else {
      gpsairborne++;
    }
    temp0 = getTempdata(address1);
    delay(100);
    temp1 = getTempdata(address2);
    delay(100);
    lightsensor();
    numbersats = gps.num_sats();
    if (nogps_ballast > 10 && ballastmode > 0) {
      stopBallast();
      n=sprintf (superbuffer, "$$ATLAS/No GPS/%d:%d:%d/%s/%s/%ld/%d/%s/%d/%ld/%d/%d/%d/%d/%d/Stopping Ballast", hour, minute, second, latbuf, lonbuf, ialt, numbersats, ascentratebuf, atfloat,float_time, temp0, temp1, light0, totalPumpMls, ballastmode);
      if (n > -1){
        n = sprintf (checksum, "*%02X\n",gps_checksum(superbuffer));
        n = sprintf (superbuffer, "%s%s", superbuffer, checksum);
        rtty_txstring(superbuffer);
        Serial.println(superbuffer);
      }
      nogps_ballast = 0;
    }
    else {
      n=sprintf (superbuffer, "$$ATLAS/No GPS/%d:%d:%d/%s/%s/%ld/%d/%s/%d/%ld/%d/%d/%d/%d/%d", hour, minute, second, latbuf, lonbuf, ialt, numbersats, ascentratebuf, atfloat,float_time, temp0, temp1, light0, totalPumpMls, ballastmode);
      if (n > -1){
            n = sprintf (checksum, "*%02X\n",gps_checksum(superbuffer));
            n = sprintf (superbuffer, "%s%s", superbuffer, checksum);
            rtty_txstring(superbuffer);
            Serial.println(superbuffer);
      }
    }
    nogps_count = 0;
    nogps_ballast++;
    Serial.println(nogps_ballast);
  }
}
