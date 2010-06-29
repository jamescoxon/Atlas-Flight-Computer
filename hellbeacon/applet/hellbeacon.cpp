// I/O Pins
#include "WProgram.h"
void helldelay();
void space();
void on();
void send(char c);
void sendmsg(char *str);
void setup();
void loop();
int ledPin =  7;    // red led connected to digital pin 6
int txPin = 3; // radio tx line
int radioPin = 4; //radio En line

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

int j, q;

void helldelay()
{
  delay(8);
  //delayMicroseconds(163);
  delayMicroseconds(170);
}

void space()
{
    int delayloop = 0;
    while(delayloop < 14)
    {
      helldelay();
      delayloop++;
    }
}

void on()
{
  digitalWrite(ledPin, HIGH) ;
  digitalWrite(txPin, HIGH) ;
  helldelay();
  digitalWrite(ledPin, LOW) ;
  digitalWrite(txPin, LOW) ;
}

void send(char c)
{
  int i ;
  if (c == ' ') {
    Serial.print(c) ;
    space();
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
      space();
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
  Serial.print("?") ;
}

void sendmsg(char *str)
{
  while (*str)
    send(*str++) ;
  Serial.println("");
}

void setup() {
  Serial.begin(4800);	     // start serial for output
  Serial.println("Starting..."); 
  pinMode(ledPin, OUTPUT);
  pinMode(txPin, OUTPUT);
  pinMode(radioPin, OUTPUT);
  digitalWrite(radioPin, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(2000);
  digitalWrite(ledPin,LOW);
}

void loop() {

  sendmsg("VVVVV,ATLAS,1,HIGH,ALTITUDE,BALLOON");
  delay(1000);
  sendmsg("M6JCX");
  delay(1000);
  sendmsg("0987654321");
  delay(1000);

}




int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

