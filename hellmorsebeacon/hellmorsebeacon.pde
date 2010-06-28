// I/O Pins
int ledPin =  7;    // red led connected to digital pin 6
int txPin = 3; // radio tx line
int radioPin = 4; //radio En line

int loopcount = 0, cycle = 0, j, q;


struct t_mtab { char c, pat; } ;

struct t_mtab morsetab[] = {
  	{'.', 106},
	{',', 115},
	{'?', 76},
	{'/', 41},
        {'-', 97},
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
#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))

#define SPEED  (15)
#define DOTLEN  (1200/SPEED)
#define DASHLEN  (3*(1200/SPEED))

struct t_htab { char c; int hellpat[5]; } ;

struct t_htab helltab[] = {

  {'1', { B00000100, B00000100, B01111100, B00000000, B00000000 } },
  {'2', { B01001000, B01100100, B01010100, B01001100, B01000000 } },
  {'3', { B01000100, B01000100, B01010100, B01010100, B00111100 } },
  {'4', { B00011100, B00010000, B00010000, B01111100, B00010000 } },
  {'5', { B01000000, B01011100, B01010100, B01010100, B00110100 } },
  {'6', { B00111100, B01010010, B01001010, B01001000, B00110000 } },
  {'7', { B01000100, B00100100, B00010100, B00001100, B00000100 } },
  {'8', { B01101100, B01011010, B01010100, B01011010, B01101100 } },
  {'9', { B00001000, B01001010, B01001010, B00101010, B00111000 } },
  {'0', { B00111000, B01100100, B01010100, B01001100, B00111000 } },
  {'A', { B01111000, B00101100, B00100100, B00101100, B01111000 } },
  {'B', { B01000100, B01111100, B01010100, B01010100, B00101000 } },
  {'C', { B00111000, B01101100, B01000100, B01000100, B00101000 } },
  {'D', { B01000100, B01111100, B01000100, B01000100, B00111000 } },
  {'E', { B01111100, B01010100, B01010100, B01000100, B01000100 } },
  {'G', { B00111000, B01101100, B01000100, B01010100, B00110100 } },
  {'H', { B01111100, B00010000, B00010000, B00010000, B01111100 } },
  {'I', { B00000000, B01000100, B01111100, B01000100, B00000000 } },
  {'J', { B01100000, B01000000, B01000000, B01000000, B01111100 } },
  {'L', { B01111100, B01000000, B01000000, B01000000, B01000000 } },
  {'M', { B01111100, B00001000, B00010000, B00001000, B01111100 } },
  {'N', { B01111100, B00000100, B00001000, B00010000, B01111100 } },
  {'O', { B00111000, B01000100, B01000100, B01000100, B00111000 } },
  {'S', { B01011000, B01010100, B01010100, B01010100, B00100100 } },
  {'T', { B00000100, B00000100, B01111100, B00000100, B00000100 } },
  {'U', { B01111100, B01000000, B01000000, B01000000, B01111100 } },
  {'V', { B01111100, B00100000, B00010000, B00001000, B00000100 } },
  {'X', { B01000100, B00101000, B00010000, B00101000, B01000100 } },
  {',', { B10000000, B10100000, B01100000, B00000000, B00000000 } }

};

#define N_HELL  (sizeof(helltab)/sizeof(helltab[0]))

void helldelay()
{
  delay(8);
  delayMicroseconds(163);
}


void on()
{
  digitalWrite(ledPin, HIGH) ;
  digitalWrite(txPin, HIGH) ;
  helldelay();
  digitalWrite(ledPin, LOW) ;
  digitalWrite(txPin, LOW) ;
}

void
dash()
{
  digitalWrite(ledPin, HIGH) ;
  digitalWrite(txPin, HIGH) ;
  delay(DASHLEN);
  digitalWrite(ledPin, LOW) ;
  digitalWrite(txPin, LOW) ;
  delay(DOTLEN) ;
}

void
dit()
{
  digitalWrite(ledPin, HIGH) ;
  digitalWrite(txPin, HIGH) ;
  delay(DOTLEN);
  digitalWrite(ledPin, LOW) ;
  digitalWrite(txPin, LOW) ;
  delay(DOTLEN);
}

void
send(char c)
{
  int i ;
  if (c == ' ') {
    Serial.print(c) ;
    delay(7*DOTLEN) ;
    return ;
  }
  for (i=0; i<N_MORSE; i++) {
    if (morsetab[i].c == c) {
      unsigned char p = morsetab[i].pat ;
      Serial.print(morsetab[i].c) ;

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
  Serial.print("?") ;
}

void
sendmsg(char *str)
{
  while (*str)
    send(*str++) ;
  Serial.println("");
}

void hellsend(char c)
{
  int i ;
  if (c == ' ') {
    Serial.print(c) ;
    delay(114);
    delayMicroseconds(282);
    return ;
  }
  for (i=0; i<N_HELL; i++) {
    if (helltab[i].c == c) {
      Serial.print(helltab[i].c) ;

      for (j=0; j<=4; j++) 
      {
        byte mask = B10000000;
        for (q=0; q<6; q++)
        {      
          if(helltab[i].hellpat[j] & mask) {
            on();
          } else {
            helldelay();
          }
          mask >>= 1;
        }
    
      }
      delay(114);
      delayMicroseconds(282);
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
  Serial.print("?") ;
}

void hellsendmsg(char *str)
{
  while (*str)
    hellsend(*str++) ;
  Serial.println("");
}

void setup()
{
  Serial.begin(4800);	     // start serial for output
  Serial.println("Starting..."); 
  pinMode(ledPin, OUTPUT);
  pinMode(txPin, OUTPUT);
  pinMode(radioPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(2000);
  digitalWrite(ledPin,LOW);
}

void loop()
{  
  //Tx Stuff
if (loopcount < 4) {
  loopcount++;
  switch(loopcount) {
  case 1:
    digitalWrite(radioPin, HIGH);
    delay(1000);
    sendmsg("VVVVV,ATLAS,1/3,HIGH,ALTITUDE,BALLOON");
    delay(1000);
    hellsendmsg("VVVVV,ATLAS,1,HIGH,ALTITUDE,BALLOON");
    digitalWrite(radioPin, LOW);
    delay(60000);
    break;
  case 2:
    digitalWrite(radioPin, HIGH);
    delay(1000);
    sendmsg("VVVVV,ATLAS,2/3,RTTY,434.075,ASCII8,50,350,0,1.5");
    delay(1000);
    hellsendmsg("VVVVV,ATLAS,1,HIGH,ALTITUDE,BALLOON");
    digitalWrite(radioPin, LOW);
    delay(60000);
    break;
  case 3:
    digitalWrite(radioPin, HIGH);
    delay(1000);
    sendmsg("VVVVV,ATLAS,3/3,WWW.SPACENEAR.US/TRACKER/");
    delay(1000);
    hellsendmsg("VVVVV,ATLAS,1,HIGH,ALTITUDE,BALLOON");
    digitalWrite(radioPin, LOW);
    delay(60000);
    break;
}
}
else if (loopcount > 3) {
    loopcount = 0;
    delay(900000);
  }
}