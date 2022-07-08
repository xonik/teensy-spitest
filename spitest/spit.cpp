/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "spit.h"
#include "pins_arduino.h"

uint8_t         txBuffer[8];
EventResponder  callbackHandlerFast;

/**********************************************************/
/*     32 bit Teensy 4.x                                  */
/**********************************************************/

#if defined(__arm__) && defined(TEENSYDUINO) && (defined(__IMXRT1052__) || defined(__IMXRT1062__))

void SPIClass::begin()
{
  hardware().clock_gate_register &= ~hardware().clock_gate_mask;

  CCM_CBCMR = 
    (CCM_CBCMR & ~(
      CCM_CBCMR_LPSPI_PODF_MASK | 
      CCM_CBCMR_LPSPI_CLK_SEL_MASK)
    ) |
    CCM_CBCMR_LPSPI_PODF(2) | 
    CCM_CBCMR_LPSPI_CLK_SEL(1); // pg 714

  uint32_t fastio = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
  *(portControlRegister(hardware().miso_pin[miso_pin_index])) = fastio;
  *(portControlRegister(hardware().mosi_pin[mosi_pin_index])) = fastio;
  *(portControlRegister(hardware().sck_pin[sck_pin_index])) = fastio;

  hardware().clock_gate_register |= hardware().clock_gate_mask;
  *(portConfigRegister(hardware().miso_pin[miso_pin_index])) = hardware().miso_mux[miso_pin_index];
  *(portConfigRegister(hardware().mosi_pin [mosi_pin_index])) = hardware().mosi_mux[mosi_pin_index];
  *(portConfigRegister(hardware().sck_pin [sck_pin_index])) = hardware().sck_mux[sck_pin_index];

  // Set the Mux pins 
  //Serial.println("SPI: Set Input select registers");
  hardware().sck_select_input_register = hardware().sck_select_val[sck_pin_index];
  hardware().miso_select_input_register = hardware().miso_select_val[miso_pin_index];
  hardware().mosi_select_input_register = hardware().mosi_select_val[mosi_pin_index];

  port().CR = LPSPI_CR_RST;

  // Lets initialize the Transmit FIFO watermark to FIFO size - 1... 
  // BUGBUG:: I assume queue of 16 for now...
  port().FCR = LPSPI_FCR_TXWATER(15);

  // We should initialize the SPI to be in a known default state.
  beginTransaction(SPISettings());
  endTransaction();
}

void SPIClass::setClockDivider_noInline(uint32_t clk) {
  // Again depreciated, but... 
  hardware().clock_gate_register |= hardware().clock_gate_mask;
  if (clk != _clock) {
    static const uint32_t clk_sel[4] = {664615384,  // PLL3 PFD1
               720000000,  // PLL3 PFD0
               528000000,  // PLL2
               396000000}; // PLL2 PFD2       

      // First save away the new settings..
      _clock = clk;

    uint32_t cbcmr = CCM_CBCMR;
    uint32_t clkhz = clk_sel[(cbcmr >> 4) & 0x03] / (((cbcmr >> 26 ) & 0x07 ) + 1);  // LPSPI peripheral clock
    
    uint32_t d, div;    
    d = _clock ? clkhz/_clock : clkhz;

    if (d && clkhz/d > _clock) d++;
    if (d > 257) d= 257;  // max div
    if (d > 2) {
      div = d-2;
    } else {
      div =0;
    }

    _ccr = LPSPI_CCR_SCKDIV(div) | LPSPI_CCR_DBT(div/2) | LPSPI_CCR_PCSSCK(div/2);

  } 
  //Serial.printf("SPI.setClockDivider_noInline CCR:%x TCR:%x\n", _ccr, port().TCR);
  port().CR = 0;
  port().CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_SAMPLE;
  port().CCR = _ccr;
  port().CR = LPSPI_CR_MEN;
}


uint8_t SPIClass::pinIsChipSelect(uint8_t pin)
{
  for (unsigned int i = 0; i < sizeof(hardware().cs_pin); i++) {
    if (pin == hardware().cs_pin[i]) return hardware().cs_mask[i];
  }
  return 0;
}

bool SPIClass::pinIsChipSelect(uint8_t pin1, uint8_t pin2)
{
  uint8_t pin1_mask, pin2_mask;
  if ((pin1_mask = (uint8_t)pinIsChipSelect(pin1)) == 0) return false;
  if ((pin2_mask = (uint8_t)pinIsChipSelect(pin2)) == 0) return false;
  //Serial.printf("pinIsChipSelect %d %d %x %x\n\r", pin1, pin2, pin1_mask, pin2_mask);
  if ((pin1_mask & pin2_mask) != 0) return false;
  return true;
}


bool SPIClass::pinIsMOSI(uint8_t pin)
{
  for (unsigned int i = 0; i < sizeof(hardware().mosi_pin); i++) {
    if (pin == hardware().mosi_pin[i]) return true;
  }
  return false;
}

bool SPIClass::pinIsMISO(uint8_t pin)
{
  for (unsigned int i = 0; i < sizeof(hardware().miso_pin); i++) {
    if (pin == hardware().miso_pin[i]) return true;
  }
  return false;
}

bool SPIClass::pinIsSCK(uint8_t pin)
{
  for (unsigned int i = 0; i < sizeof(hardware().sck_pin); i++) {
    if (pin == hardware().sck_pin[i]) return true;
  }
  return false;
}

// setCS() is not intended for use from normal Arduino programs/sketches.
uint8_t SPIClass::setCS(uint8_t pin)
{
  for (unsigned int i = 0; i < sizeof(hardware().cs_pin); i++) {
    if (pin == hardware().cs_pin[i]) {
      *(portConfigRegister(pin)) = hardware().cs_mux[i];
      if (hardware().pcs_select_input_register[i])
        *hardware().pcs_select_input_register[i] = hardware().pcs_select_val[i];
      return hardware().cs_mask[i];
    }
  }
  return 0;
}

void SPIClass::setMOSI(uint8_t pin)
{
  if (pin != hardware().mosi_pin[mosi_pin_index]) {
    for (unsigned int i = 0; i < sizeof(hardware().mosi_pin); i++) {
      if (pin == hardware().mosi_pin[i] ) {
        if (hardware().clock_gate_register & hardware().clock_gate_mask) {
          // BUGBUG:: Unclear what to do with previous pin as there is no unused setting like t3.x
          uint32_t fastio = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
          *(portControlRegister(hardware().mosi_pin[i])) = fastio;
          *(portConfigRegister(hardware().mosi_pin [i])) = hardware().mosi_mux[i];
          hardware().mosi_select_input_register = hardware().mosi_select_val[i];
        } 
        mosi_pin_index = i;
        return;
      }
    }
  }
}

void SPIClass::setMISO(uint8_t pin)
{
  if (pin != hardware().miso_pin[miso_pin_index]) {
    for (unsigned int i = 0; i < sizeof(hardware().miso_pin); i++) {
      if (pin == hardware().miso_pin[i] ) {
        if (hardware().clock_gate_register & hardware().clock_gate_mask) {
          // BUGBUG:: Unclear what to do with previous pin as there is no unused setting like t3.x
          uint32_t fastio = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
          *(portControlRegister(hardware().miso_pin[i])) = fastio;
          *(portConfigRegister(hardware().miso_pin[i])) = hardware().miso_mux[i];
          hardware().miso_select_input_register = hardware().miso_select_val[i];
        } 
        miso_pin_index = i;
        return;
      }
    }
  }
}

void SPIClass::setSCK(uint8_t pin)
{
  if (pin != hardware().sck_pin[sck_pin_index]) {
    for (unsigned int i = 0; i < sizeof(hardware().sck_pin); i++) {
      if (pin == hardware().sck_pin[i] ) {
        if (hardware().clock_gate_register & hardware().clock_gate_mask) {
          // BUGBUG:: Unclear what to do with previous pin as there is no unused setting like t3.x
          uint32_t fastio = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
          *(portControlRegister(hardware().sck_pin[i])) = fastio;
          *(portConfigRegister(hardware().sck_pin [i])) = hardware().sck_mux[i];
          hardware().sck_select_input_register = hardware().sck_select_val[i];
        } 
        sck_pin_index = i;
        return;
      }
    }
  }
}


void SPIClass::setBitOrder(uint8_t bitOrder)
{
  hardware().clock_gate_register |= hardware().clock_gate_mask;

  if (bitOrder == LSBFIRST) {
    port().TCR |= LPSPI_TCR_LSBF;
  } else {
    port().TCR &= ~LPSPI_TCR_LSBF;
  }
}

void SPIClass::setDataMode(uint8_t dataMode)
{
  hardware().clock_gate_register |= hardware().clock_gate_mask;
  //SPCR = (SPCR & ~SPI_MODE_MASK) | dataMode;

  // Handle Data Mode
  uint32_t tcr = port().TCR & ~(LPSPI_TCR_CPOL | LPSPI_TCR_CPHA);

  if (dataMode & 0x08) tcr |= LPSPI_TCR_CPOL;

  // Note: On T3.2 when we set CPHA it also updated the timing.  It moved the 
  // PCS to SCK Delay Prescaler into the After SCK Delay Prescaler  
  if (dataMode & 0x04) tcr |= LPSPI_TCR_CPHA; 

  // Save back out
  port().TCR = tcr;

}


void _spi_dma_rxISR0(void) {SPI.dma_rxisr();}

// NOTE pin definitions are in the order MISO, MOSI, SCK, CS 
// With each group, having pin number[n], setting[n], INPUT_SELECT_MUX settings[n], SELECT INPUT register
#if defined(ARDUINO_TEENSY41)
const SPIClass::SPI_Hardware_t  SPIClass::spiclass_lpspi4_hardware = {
  CCM_CCGR1, CCM_CCGR1_LPSPI4(CCM_CCGR_ON),
  DMAMUX_SOURCE_LPSPI4_TX, DMAMUX_SOURCE_LPSPI4_RX, _spi_dma_rxISR0,
  12, 255,  // MISO
  3 | 0x10, 0,
  0, 0,
  IOMUXC_LPSPI4_SDI_SELECT_INPUT,
  11, 255, // MOSI
  3 | 0x10, 0,
  0, 0, 
  IOMUXC_LPSPI4_SDO_SELECT_INPUT,
  13, 255, // SCK
  3 | 0x10, 0,
  0, 0,
  IOMUXC_LPSPI4_SCK_SELECT_INPUT,
  10, 37, 36, // CS
  3 | 0x10, 2 | 0x10, 2 | 0x10, 
  1, 2, 3,
  0, 0, 0,
  &IOMUXC_LPSPI4_PCS0_SELECT_INPUT, 0, 0
};
#else
const SPIClass::SPI_Hardware_t  SPIClass::spiclass_lpspi4_hardware = {
  CCM_CCGR1, CCM_CCGR1_LPSPI4(CCM_CCGR_ON),
  DMAMUX_SOURCE_LPSPI4_TX, DMAMUX_SOURCE_LPSPI4_RX, _spi_dma_rxISR0,
  12, 
  3 | 0x10,
  0,
  IOMUXC_LPSPI4_SDI_SELECT_INPUT,
  11,
  3 | 0x10,
  0,
  IOMUXC_LPSPI4_SDO_SELECT_INPUT,
  13,
  3 | 0x10,
  0, 
  IOMUXC_LPSPI4_SCK_SELECT_INPUT,
  10,
  3 | 0x10,
  1, 
  0,
  &IOMUXC_LPSPI4_PCS0_SELECT_INPUT
};
#endif

SPIClass SPI((uintptr_t)&IMXRT_LPSPI4_S, (uintptr_t)&SPIClass::spiclass_lpspi4_hardware);

#if defined(__IMXRT1062__)
// T4 has two other possible SPI objects...
void _spi_dma_rxISR1(void) {SPI1.dma_rxisr();}

#if defined(ARDUINO_TEENSY41)
const SPIClass::SPI_Hardware_t  SPIClass::spiclass_lpspi3_hardware = {
  CCM_CCGR1, CCM_CCGR1_LPSPI3(CCM_CCGR_ON),
  DMAMUX_SOURCE_LPSPI3_TX, DMAMUX_SOURCE_LPSPI3_RX, _spi_dma_rxISR1,
  1, 39,
  7 | 0x10, 2 | 0x10,
  0, 1,
  IOMUXC_LPSPI3_SDI_SELECT_INPUT,
  26, 255,
  2 | 0x10, 0,
  1, 0,
  IOMUXC_LPSPI3_SDO_SELECT_INPUT,
  27, 255,
  2 | 0x10, 0,
  1,  0,
  IOMUXC_LPSPI3_SCK_SELECT_INPUT,
  0, 38, 255,
  7 | 0x10, 2 | 0x10, 0,
  1, 1, 0,
  0, 1, 0,
  &IOMUXC_LPSPI3_PCS0_SELECT_INPUT, &IOMUXC_LPSPI3_PCS0_SELECT_INPUT, 0
};
#else
const SPIClass::SPI_Hardware_t  SPIClass::spiclass_lpspi3_hardware = {
  CCM_CCGR1, CCM_CCGR1_LPSPI3(CCM_CCGR_ON),
  DMAMUX_SOURCE_LPSPI3_TX, DMAMUX_SOURCE_LPSPI3_RX, _spi_dma_rxISR1,
  1, 
  7 | 0x10,
  0,
  IOMUXC_LPSPI3_SDI_SELECT_INPUT,
  26,
  2 | 0x10,
  1,
  IOMUXC_LPSPI3_SDO_SELECT_INPUT,
  27,
  2 | 0x10,
  1, 
  IOMUXC_LPSPI3_SCK_SELECT_INPUT,
  0,
  7 | 0x10,
  1,
  0, 
  &IOMUXC_LPSPI3_PCS0_SELECT_INPUT
};
#endif
SPIClass SPI1((uintptr_t)&IMXRT_LPSPI3_S, (uintptr_t)&SPIClass::spiclass_lpspi3_hardware);

void _spi_dma_rxISR2(void) {SPI2.dma_rxisr();}

#if defined(ARDUINO_TEENSY41)
const SPIClass::SPI_Hardware_t  SPIClass::spiclass_lpspi1_hardware = {
  CCM_CCGR1, CCM_CCGR1_LPSPI1(CCM_CCGR_ON),
  DMAMUX_SOURCE_LPSPI1_TX, DMAMUX_SOURCE_LPSPI1_RX, _spi_dma_rxISR1,
  42, 54,
  4 | 0x10, 3 | 0x10,
  1, 0,
  IOMUXC_LPSPI1_SDI_SELECT_INPUT,
  43, 50,
  4 | 0x10, 3 | 0x10,
  1, 0,
  IOMUXC_LPSPI1_SDO_SELECT_INPUT,
  45, 49,
  4 | 0x10, 3 | 0x10,
  1, 0, 
  IOMUXC_LPSPI1_SCK_SELECT_INPUT,
  44, 255, 255,
  4 | 0x10, 0, 0,
  1, 0, 0,
  0, 0, 0,
  &IOMUXC_LPSPI1_PCS0_SELECT_INPUT, 0, 0
};
#else
const SPIClass::SPI_Hardware_t  SPIClass::spiclass_lpspi1_hardware = {
  CCM_CCGR1, CCM_CCGR1_LPSPI1(CCM_CCGR_ON),
  DMAMUX_SOURCE_LPSPI1_TX, DMAMUX_SOURCE_LPSPI1_RX, _spi_dma_rxISR1,
  34, 
  4 | 0x10,
  1,
  IOMUXC_LPSPI1_SDI_SELECT_INPUT,
  35,
  4 | 0x10,
  1,
  IOMUXC_LPSPI1_SDO_SELECT_INPUT,
  37,
  4 | 0x10,
  1,
  IOMUXC_LPSPI1_SCK_SELECT_INPUT,
  36,
  4 | 0x10,
  1,
  0,
  &IOMUXC_LPSPI1_PCS0_SELECT_INPUT
};
#endif
SPIClass SPI2((uintptr_t)&IMXRT_LPSPI1_S, (uintptr_t)&SPIClass::spiclass_lpspi1_hardware);
#endif

//SPIClass SPI(&IMXRT_LPSPI4_S, &spiclass_lpspi4_hardware);

void SPIClass::usingInterrupt(IRQ_NUMBER_t interruptName)
{
  uint32_t n = (uint32_t)interruptName;

  if (n >= NVIC_NUM_INTERRUPTS) return;

  //Serial.print("usingInterrupt ");
  //Serial.println(n);
  interruptMasksUsed |= (1 << (n >> 5));
  interruptMask[n >> 5] |= (1 << (n & 0x1F));
  //Serial.printf("interruptMasksUsed = %d\n", interruptMasksUsed);
  //Serial.printf("interruptMask[0] = %08X\n", interruptMask[0]);
  //Serial.printf("interruptMask[1] = %08X\n", interruptMask[1]);
  //Serial.printf("interruptMask[2] = %08X\n", interruptMask[2]);
}

void SPIClass::notUsingInterrupt(IRQ_NUMBER_t interruptName)
{
  uint32_t n = (uint32_t)interruptName;
  if (n >= NVIC_NUM_INTERRUPTS) return;
  interruptMask[n >> 5] &= ~(1 << (n & 0x1F));
  if (interruptMask[n >> 5] == 0) {
    interruptMasksUsed &= ~(1 << (n >> 5));
  }
}


void SPIClass::end() {
  // only do something if we have begun
  if (hardware().clock_gate_register & hardware().clock_gate_mask) {
    port().CR = 0;  // turn off the enable
    pinMode(hardware().miso_pin[miso_pin_index], INPUT_DISABLE);
    pinMode(hardware().mosi_pin[mosi_pin_index], INPUT_DISABLE);
    pinMode(hardware().sck_pin[sck_pin_index], INPUT_DISABLE);
  }
}

//=============================================================================
// ASYNCH Support
//=============================================================================
//=========================================================================
// Try Transfer using DMA.
//=========================================================================
#ifdef SPI_HAS_TRANSFER_ASYNC
static uint8_t bit_bucket;
#define dontInterruptAtCompletion(dmac) (dmac)->TCD->CSR &= ~DMA_TCD_CSR_INTMAJOR

//=========================================================================
// Init the DMA channels
//=========================================================================
bool SPIClass::initDMAChannels() {
  // Allocate our channels. 
  _dmaTX = new DMAChannel();
  if (_dmaTX == nullptr) {
    return false;
  }

  _dmaRX = new DMAChannel();
  if (_dmaRX == nullptr) {
    delete _dmaTX; // release it
    _dmaTX = nullptr; 
    return false;
  }

  // Let's setup the RX chain
  _dmaRX->disable();
  _dmaRX->source((volatile uint8_t&)port().RDR);
  _dmaRX->disableOnCompletion();
  _dmaRX->triggerAtHardwareEvent(hardware().rx_dma_channel);
  _dmaRX->attachInterrupt(hardware().dma_rxisr);
  _dmaRX->interruptAtCompletion();

  // We may be using settings chain here so lets set it up. 
  // Now lets setup TX chain.  Note if trigger TX is not set
  // we need to have the RX do it for us.
  _dmaTX->disable();
  _dmaTX->destination((volatile uint8_t&)port().TDR);
  _dmaTX->disableOnCompletion();

  if (hardware().tx_dma_channel) {
    _dmaTX->triggerAtHardwareEvent(hardware().tx_dma_channel);
  } else {
//    Serial.printf("SPI InitDMA tx triger by RX: %x\n", (uint32_t)_dmaRX);
      _dmaTX->triggerAtTransfersOf(*_dmaRX);
  }


  _dma_state = DMAState::idle;  // Should be first thing set!
  return true;
}

//=========================================================================
// Main Async Transfer function
//=========================================================================
#ifndef TRANSFER_COUNT_FIXED
inline void DMAChanneltransferCount(DMAChannel * dmac, unsigned int len) {
  // note does no validation of length...
  DMABaseClass::TCD_t *tcd = dmac->TCD;
  if (!(tcd->BITER & DMA_TCD_BITER_ELINK)) {
    tcd->BITER = len & 0x7fff;
  } else {
    tcd->BITER = (tcd->BITER & 0xFE00) | (len & 0x1ff);
  }
  tcd->CITER = tcd->BITER; 
}
#else 
inline void DMAChanneltransferCount(DMAChannel * dmac, unsigned int len) {
  dmac->transferCount(len);
}
#endif
#ifdef DEBUG_DMA_TRANSFERS
void dumpDMA_TCD(DMABaseClass *dmabc)
{
  Serial4.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  Serial4.printf("SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n", (uint32_t)dmabc->TCD->SADDR,
    dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR, 
    dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}
#endif

bool SPIClass::transferSetup() {
  if (_dma_state == DMAState::notAllocated) {
    if (!initDMAChannels())
      return false;
  }

  if (_dma_state == DMAState::active)
    return false; // already active  
  _dmaTX->TCD->ATTR_DST = 0;    // Make sure set for 8 bit mode
  _dmaRX->TCD->ATTR_SRC = 0;    //Make sure set for 8 bit mode...  
  
  DMAChanneltransferCount(_dmaRX, 8);
  _dmaRX->destination((uint8_t&)bit_bucket);

  // Make sure port is in 8 bit mode and clear watermark
  port().TCR = (port().TCR & ~(LPSPI_TCR_FRAMESZ(31))) | LPSPI_TCR_FRAMESZ(7);  
  port().FCR = 0; 

  // Lets try to output the first byte to make sure that we are in 8 bit mode...
  port().DER = LPSPI_DER_TDDE | LPSPI_DER_RDDE; //enable DMA on both TX and RX

  // We reuse the same buffer and update it in the ISR for multi-block transfers. Seems
  // to work ok.
  _dmaTX->sourceBuffer((uint8_t*)txBuffer, 8); 
}

bool SPIClass::transfer(const void *buf, size_t count) {

  digitalWriteFast(0,HIGH); // 1 
  // lets clear cache before we update sizes...
  //if ((uint32_t)buf >= 0x20200000u)  arm_dcache_flush((uint8_t *)buf, count);
  digitalWriteFast(0,LOW);   
  // 1 instr = approx 1.7nS
  yield();
  port().SR = 0x3f00; // clear out all of the other status...
  digitalWriteFast(0,HIGH); // 3 
  _dmaRX->enable();
  digitalWriteFast(0,LOW);  
  _dmaTX->enable();
  digitalWriteFast(0,HIGH); // 4 
  _dma_state = DMAState::active;
  digitalWriteFast(0,LOW);  
  return true;
}

uint8_t flip = 0;

//-------------------------------------------------------------------------
// DMA RX ISR
//-------------------------------------------------------------------------
void SPIClass::dma_rxisr(void) {
  digitalWriteFast(0,HIGH); //0 
  _dmaRX->clearInterrupt();
  //_dmaTX->clearComplete();

  //port().FCR = LPSPI_FCR_TXWATER(15); // _spi_fcr_save; // restore the FSR status... 
  //port().DER = 0;   // DMA no longer doing TX (or RX)

  port().CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF;   // actually clear both...
  port().SR = 0x3f00; // clear out all of the other status...

  _dma_state = DMAState::completed;   // set back to 1 in case our call wants to start up dma again
  
  digitalWriteFast(0,LOW);

  if(flip == 0) {
    flip = 0xFF;
  } else {
    flip = 0;
  }
  for(uint8_t i = 0; i<8; i++){
    txBuffer[i] = flip; 
  }

  transfer((void *)txBuffer, 8);
}
#endif // SPI_HAS_TRANSFER_ASYNC

#endif
