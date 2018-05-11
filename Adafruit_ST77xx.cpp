/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ST77xx.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

inline uint16_t swapcolor(uint16_t x) { 
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

#if defined (SPI_HAS_TRANSACTION)
  static SPISettings mySPISettings;
#elif defined (__AVR__) || defined(CORE_TEENSY)
  static uint8_t SPCRbackup;
  static uint8_t mySPCR;
#endif


void ISpiDriver::init()
{
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);

  #if defined(USE_FAST_IO)
    csport    = portOutputRegister(digitalPinToPort(_cs));
    dcport    = portOutputRegister(digitalPinToPort(_dc));
    cspinmask = digitalPinToBitMask(_cs);
    dcpinmask = digitalPinToBitMask(_dc);
  #endif
}

void HwSpiDriver::init()
{
  ISpiDriver::init();
  #if defined (SPI_HAS_TRANSACTION)
    SPI.begin();
    mySPISettings = SPISettings(24000000, MSBFIRST, SPI_MODE0);
  #elif defined (__AVR__) || defined(CORE_TEENSY)
    SPCRbackup = SPCR;
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.setDataMode(SPI_MODE0);
    mySPCR = SPCR; // save our preferred state
    //Serial.print("mySPCR = 0x"); Serial.println(SPCR, HEX);
    SPCR = SPCRbackup;  // then restore
  #elif defined (__SAM3X8E__)
    SPI.begin();
    SPI.setClockDivider(5/*21*/); //4MHz
    SPI.setDataMode(SPI_MODE0);
  #endif
}

void BitbangSpiDriver::init()
{
  ISpiDriver::init();
  pinMode(_sclk, OUTPUT);
  pinMode(_sid , OUTPUT);
  digitalWrite(_sclk, LOW);
  digitalWrite(_sid, LOW);

  #if defined(USE_FAST_IO)
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    dataport    = portOutputRegister(digitalPinToPort(_sid));
    clkpinmask  = digitalPinToBitMask(_sclk);
    datapinmask = digitalPinToBitMask(_sid);
  #endif
}

// Constructor when using software SPI.  All output pins are configurable.
/*Adafruit_ST77xx::Adafruit_ST77xx(int8_t cs, int8_t dc, int8_t sid, int8_t sclk, int8_t rst) 
  : Adafruit_GFX(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160)
{
  _cs   = cs;
  _dc   = dc;
  _sid  = sid;
  _sclk = sclk;
  _rst  = rst;
  _hwSPI = false;
}

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ST77xx::Adafruit_ST77xx(int8_t cs, int8_t dc, int8_t rst) 
  : Adafruit_GFX(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  _hwSPI = true;
  _sid  = _sclk = -1;
}*/

void Adafruit_ST77xx::spiwrite(uint8_t c) {

  _spi.write(c);
}

void HwSpiDriver::write(uint8_t c) const {
  #if defined (SPI_HAS_TRANSACTION)
    SPI.transfer(c);
  #elif defined (__AVR__) || defined(CORE_TEENSY)
    SPCRbackup = SPCR;
    SPCR = mySPCR;
    SPI.transfer(c);
    SPCR = SPCRbackup;
  #elif defined (__arm__)
    SPI.setClockDivider(21); //4MHz
    SPI.setDataMode(SPI_MODE0);
    SPI.transfer(c);
  #endif
}

void HwSpiDriver::BEGIN_TRANSACTION() const
{
  #if defined(SPI_HAS_TRANSACTION)
    SPI.beginTransaction(mySPISettings);
  #endif
}

void HwSpiDriver::END_TRANSACTION() const
{
  #if defined(SPI_HAS_TRANSACTION)
    SPI.endTransaction();
  #endif
}


void BitbangSpiDriver::write(uint8_t c) const {
  // Fast SPI bitbang swiped from LPD8806 library
  for(uint8_t bit = 0x80; bit; bit >>= 1) {
#if defined(USE_FAST_IO)
    if(c & bit) *dataport |=  datapinmask;
    else        *dataport &= ~datapinmask;
    *clkport |=  clkpinmask;
    *clkport &= ~clkpinmask;
#else
    if(c & bit) digitalWrite(_sid, HIGH);
    else        digitalWrite(_sid, LOW);
    digitalWrite(_sclk, HIGH);
    digitalWrite(_sclk, LOW);
#endif
  }
}


void Adafruit_ST77xx::writecommand(uint8_t c) {

  _spi.DC_LOW();
  _spi.CS_LOW();
  _spi.BEGIN_TRANSACTION();

  spiwrite(c);

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}


void Adafruit_ST77xx::writedata(uint8_t c) {
  _spi.BEGIN_TRANSACTION();
  _spi.DC_HIGH();
  _spi.CS_LOW();
    
  spiwrite(c);

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ST77xx::displayInit(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   //   If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


// Initialization code common to all ST77XX displays
void Adafruit_ST77xx::commonInit(const uint8_t *cmdList) {
  _ystart = _xstart = 0;
  _colstart  = _rowstart = 0; // May be overridden in init func

  _spi.init();

  // toggle RST low to reset; CS low so it'll listen to us
  _spi.CS_LOW();
  if (_spi._rst != -1) {
    pinMode(_spi._rst, OUTPUT);
    digitalWrite(_spi._rst, HIGH);
    delay(500);
    digitalWrite(_spi._rst, LOW);
    delay(500);
    digitalWrite(_spi._rst, HIGH);
    delay(500);
  }

  if(cmdList) 
    displayInit(cmdList);
}



void Adafruit_ST77xx::setRotation(uint8_t m) {

  writecommand(ST77XX_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB);

     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     writedata(ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);

     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  case 2:
     writedata(ST77XX_MADCTL_RGB);
 
     _xstart = _colstart;
     _ystart = _rowstart;
     break;

   case 3:
     writedata(ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);

     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  }
}

void Adafruit_ST77xx::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,
 uint8_t y1) {

  uint16_t x_start = x0 + _xstart, x_end = x1 + _xstart;
  uint16_t y_start = y0 + _ystart, y_end = y1 + _ystart;
  

  writecommand(ST77XX_CASET); // Column addr set
  writedata(x_start >> 8);
  writedata(x_start & 0xFF);     // XSTART 
  writedata(x_end >> 8);
  writedata(x_end & 0xFF);     // XEND

  writecommand(ST77XX_RASET); // Row addr set
  writedata(y_start >> 8);
  writedata(y_start & 0xFF);     // YSTART
  writedata(y_end >> 8);
  writedata(y_end & 0xFF);     // YEND

  writecommand(ST77XX_RAMWR); // write to RAM
}


void Adafruit_ST77xx::pushColor(uint16_t color) {
  _spi.BEGIN_TRANSACTION();
  _spi.DC_HIGH();
  _spi.CS_LOW();

  spiwrite(color >> 8);
  spiwrite(color);

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}

void Adafruit_ST77xx::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

  _spi.BEGIN_TRANSACTION();
  _spi.DC_HIGH();
  _spi.CS_LOW();

  spiwrite(color >> 8);
  spiwrite(color);

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}


void Adafruit_ST77xx::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  _spi.BEGIN_TRANSACTION();
  _spi.DC_HIGH();
  _spi.CS_LOW();

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}


void Adafruit_ST77xx::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  fillRect(x, y, w, 1, color);
  /*setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;

  _spi.BEGIN_TRANSACTION();
  _spi.DC_HIGH();
  _spi.CS_LOW();

  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();*/
}



void Adafruit_ST77xx::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}



// fill a rectangle
void Adafruit_ST77xx::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  _spi.BEGIN_TRANSACTION();

  _spi.DC_HIGH();
  _spi.CS_LOW();
  for (int32_t i = w*h; i>0; --i) {
    spiwrite(hi);
    spiwrite(lo);
  }
  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}


void Adafruit_ST77xx::invertDisplay(boolean i) {
  writecommand(i ? ST77XX_INVON : ST77XX_INVOFF);
}


/******** low level bit twiddling **********/


void ISpiDriver::CS_HIGH() const {
#if defined(USE_FAST_IO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
}

void ISpiDriver::CS_LOW() const {
#if defined(USE_FAST_IO)
  *csport &= ~cspinmask;
#else
  digitalWrite(_cs, LOW);
#endif
}

void ISpiDriver::DC_HIGH() const {
#if defined(USE_FAST_IO)
  *dcport |= dcpinmask;
#else
  digitalWrite(_dc, HIGH);
#endif
}

void ISpiDriver::DC_LOW() const {
#if defined(USE_FAST_IO)
  *dcport &= ~dcpinmask;
#else
  digitalWrite(_dc, LOW);
#endif
}



////////// stuff not actively being used, but kept for posterity
/*

 uint8_t Adafruit_ST77xx::spiread(void) {
 uint8_t r = 0;
 if (_sid > 0) {
 r = shiftIn(_sid, _sclk, MSBFIRST);
 } else {
 //SID_DDR &= ~_BV(SID);
 //int8_t i;
 //for (i=7; i>=0; i--) {
 //  SCLK_PORT &= ~_BV(SCLK);
 //  r <<= 1;
 //  r |= (SID_PIN >> SID) & 0x1;
 //  SCLK_PORT |= _BV(SCLK);
 //}
 //SID_DDR |= _BV(SID);
 
 }
 return r;
 }
 
 
 void Adafruit_ST77xx::dummyclock(void) {
 
 if (_sid > 0) {
 digitalWrite(_sclk, LOW);
 digitalWrite(_sclk, HIGH);
 } else {
 // SCLK_PORT &= ~_BV(SCLK);
 //SCLK_PORT |= _BV(SCLK);
 }
 }
 uint8_t Adafruit_ST77xx::readdata(void) {
 *portOutputRegister(rsport) |= rspin;
 
 *portOutputRegister(csport) &= ~ cspin;
 
 uint8_t r = spiread();
 
 *portOutputRegister(csport) |= cspin;
 
 return r;
 
 } 
 
 uint8_t Adafruit_ST77xx::readcommand8(uint8_t c) {
 digitalWrite(_rs, LOW);
 
 *portOutputRegister(csport) &= ~ cspin;
 
 spiwrite(c);
 
 digitalWrite(_rs, HIGH);
 pinMode(_sid, INPUT); // input!
 digitalWrite(_sid, LOW); // low
 spiread();
 uint8_t r = spiread();
 
 
 *portOutputRegister(csport) |= cspin;
 
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 
 uint16_t Adafruit_ST77xx::readcommand16(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Adafruit_ST77xx::readcommand32(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
