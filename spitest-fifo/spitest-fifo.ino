//No rights given, no responsibilities taken, but the code is here.
// https://forum.pjrc.com/threads/67247-Teensy-4-0-DMA-SPI/page2
#include "spit.h"

uint8_t repeats = 0;
volatile  uint8_t  sendState = 0;

EventResponder  callbackHandlerSlow;
SPISettings     fastSettings(8000000, MSBFIRST, SPI_MODE0);
SPISettings     slowSettings(2000000, MSBFIRST, SPI_MODE0);

// TODO: 
// Test
// - speed of loop without SPI
// - speed of loop with SPI
// - speed of loop with "normal" SPI
// - speed of consecutive writes with speed changes.
// - Use clock, do X repeats of something and print time.
void setup() {
  pinMode (0, OUTPUT);
  //pinMode (10, OUTPUT);
  //pinMode (36, OUTPUT);
  //pinMode (37, OUTPUT);
  Serial.begin(1000000);
  Serial.println("SPI Starting");

 
  for (int y = 0; y < 9; y++)  {           
    if((y % 2) == 0){
      txBuffer[y] = 0x555555;
    } else {
      txBuffer[y] = 0x555555; 
    }
  }

  SPI.begin();   
}

uint8_t dat = 0;

void loop() {
  if(dat==0) {
    dat=1;  
    digitalWriteFast(0,HIGH);
  } else {
    dat=0;
    digitalWriteFast(0,LOW);
  }
 

  if(sendState == 0 )  { 
    SPI.beginTransaction(fastSettings);
    repeats = 0;  
    sendFast();
  } else if(sendState == 2) {
    sendSlow();
  }
}

uint8_t csp = 11;

void sendFast() {  
  //Send txBuffer to display using SPI DMA
  sendState = 1;
 // digitalWriteFast(10,HIGH); 

  SPI.transfer24((void *)txBuffer, 9, 10); 
  sendState = 2;  
}

/*
void callbackFast(EventResponderRef eventResponder) {
  //end screen update
  
  //digitalWriteFast(10,LOW); 

  if(repeats < 64) {
    repeats++;
    sendFast();
  } else {
    sendState = 2;  
    SPI.endTransaction();
  }
}*/

void sendSlow() {  
  //Send txBuffer to display using SPI DMA
  sendState = 3;
  //digitalWriteFast(10,HIGH); 
  //SPI.setCS(36);
  SPI.beginTransaction(slowSettings);
  SPI.transfer24((void *)txBuffer, 9, 36); 
  sendState = 0;  
}

/*
void callbackSlow(EventResponderRef eventResponder) {
  //end screen update
  SPI.endTransaction();
  //digitalWriteFast(10,LOW); 
  sendState = 0;  
}
*/

/*
void clearDisplay() {
  digitalWriteFast(10,HIGH);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x20 | Vcom);
  SPI.transfer(0x00);
  digitalWriteFast(10,LOW); //end screen update
}
*/
