#include <Adafruit_STMPE610.h>

//No rights given, no responsibilities taken, but the code is here.
// https://forum.pjrc.com/threads/67247-Teensy-4-0-DMA-SPI/page2

// Copy fork of SPI to /Applications/Teensyduino.app/Contents/Java/hardware/teensy/avr/libraries/SPI
#include <SPI.h>

volatile  uint8_t sendState = 0;
uint32_t         txBuffer[9];

SPISettings     fastSettings(50000000, MSBFIRST, SPI_MODE0);
SPISettings     slowSettings(4000000, MSBFIRST, SPI_MODE0);

// TODO: 
// Test
// - speed of loop without SPI
// - speed of loop with SPI
// - speed of loop with "normal" SPI
// - speed of consecutive writes with speed changes.
// - Use clock, do X repeats of something and print time.
void setup() {
  pinMode (0, OUTPUT);
  pinMode (1, OUTPUT);
  //pinMode (10, OUTPUT);
  //pinMode (36, OUTPUT);
  //pinMode (37, OUTPUT);
  Serial.begin(1000000);
  Serial.println("SPI Starting");

  // Attach interrupt handler for SPI
  //attachInterruptVector(IRQ_LPSPI4, &spi_isr4);
  //NVIC_ENABLE_IRQ(IRQ_LPSPI4);
  SPI.enableInterrupt(&spi_isr4);
 
  for (int y = 0; y < 9; y++)  {           
    if((y % 2) == 0){
      txBuffer[y] = 0x555555;
    } else {
      txBuffer[y] = 0x555555; 
    }
  }

  SPI.begin();   

  
}

void spi_isr4(void) {
  // Check transfer complete interrupt
  if(SPI.hasInterruptFlagSet(LPSPI_SR_TCF)) {
    // clear interrupt
    SPI.clearInterruptFlag(LPSPI_SR_TCF);
    sendState++;
    if(sendState == 4) {
      sendState = 0;
    }

    digitalWriteFast(0,HIGH);
    digitalWriteFast(0,LOW);
  }
}

uint8_t dat = 0;

void loop() {
  digitalWriteFast(1,HIGH);
  digitalWriteFast(1,LOW); 

  if(sendState == 0 )  { 
    sendFast();
  } else if(sendState == 2) {
    sendSlow();
  }
}

uint8_t csp = 11;

void sendFast() {  
  //Send txBuffer to display using SPI DMA
  sendState = 1;
  SPI.beginTransaction(fastSettings);
  SPI.send24((void *)txBuffer, 8, 10);   
}

void sendSlow() {  
  //Send txBuffer to display using SPI DMA
  sendState = 3;
  //digitalWriteFast(10,HIGH); 
  //SPI.setCS(36);
  SPI.beginTransaction(slowSettings);
  SPI.send24((void *)txBuffer, 2, 36);  
}
