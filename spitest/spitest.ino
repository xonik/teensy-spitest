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
  pinMode (10, OUTPUT);
  SPI.begin(); 
  // Setup the SPI DMA callback
  callbackHandlerFast.attachImmediate(&callbackFast);
  callbackHandlerSlow.attachImmediate(&callbackSlow);  

  for (int y = 0; y < 8; y++)  {           
    if((y % 2) == 0){
      txBuffer[y] = 0;
    } else {
      txBuffer[y] = 0xFF; 
    }
  }

  SPI.transferSetup();
}

uint8_t dat = 0;

void loop() {
  /*if(dat==0) {
    dat=1;  
    digitalWriteFast(0,HIGH);
  } else {
    dat=0;
    digitalWriteFast(0,LOW);
  }*/
 

  if(sendState == 0 )  { 
    SPI.beginTransaction(fastSettings);
    repeats = 0;  
    sendFast();
  } else if(sendState == 2) {
    sendSlow();
  }
}


void sendFast() {  
  //Send txBuffer to display using SPI DMA
  sendState = 1;
 // digitalWriteFast(10,HIGH); 

  SPI.transfer((void *)txBuffer, 8); 
}

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
}

void sendSlow() {  
  //Send txBuffer to display using SPI DMA
  sendState = 3;
  //digitalWriteFast(10,HIGH); 
  SPI.beginTransaction(slowSettings);
  SPI.transfer((void *)txBuffer, 8); 
}

void callbackSlow(EventResponderRef eventResponder) {
  //end screen update
  SPI.endTransaction();
  //digitalWriteFast(10,LOW); 
  sendState = 0;  
}

/*
void clearDisplay() {
  digitalWriteFast(10,HIGH);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x20 | Vcom);
  SPI.transfer(0x00);
  digitalWriteFast(10,LOW); //end screen update
}
*/
