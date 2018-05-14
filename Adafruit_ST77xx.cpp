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
  pinMode(_cs, OUTPUT);

  #if defined(USE_FAST_IO)
    csport    = portOutputRegister(digitalPinToPort(_cs));
    cspinmask = digitalPinToBitMask(_cs);
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

void Adafruit_ST77xx::spiwrite(uint8_t c) {

  _spi.write(&c, 1);
}

void HwSpiDriver::write(uint8_t const* buf, size_t bufSize) const {
  // XXX: SPI.transfer modifies the buffer :(
  #if defined (SPI_HAS_TRANSACTION)
    SPI.transfer(const_cast<uint8_t*>(buf), bufSize);
  #elif defined (__AVR__) || defined(CORE_TEENSY)
    SPCRbackup = SPCR;
    SPCR = mySPCR;
    SPI.transfer(const_cast<uint8_t*>(buf), bufSize);
    SPCR = SPCRbackup;
  #elif defined (__arm__)
    SPI.setClockDivider(21); //4MHz
    SPI.setDataMode(SPI_MODE0);
    SPI.transfer(const_cast<uint8_t*>(buf), bufSize);
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


void BitbangSpiDriver::write(uint8_t const* buf, size_t bufSize) const {
  // Fast SPI bitbang swiped from LPD8806 library
  for (; bufSize>0; --bufSize, ++buf)
  {
    uint8_t const c = *buf;
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
}


void Adafruit_ST77xx::writecommand(uint8_t c) {

  DC_LOW();
  _spi.CS_LOW();
  _spi.BEGIN_TRANSACTION();

  spiwrite(c);

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}


void Adafruit_ST77xx::writedata(uint8_t c) {
  _spi.BEGIN_TRANSACTION();
  DC_HIGH();
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

  #if defined(USE_FAST_IO)
    dcport    = portOutputRegister(digitalPinToPort(_dc));
    dcpinmask = digitalPinToBitMask(_dc);
  #endif

  pinMode(_dc, OUTPUT);
  _spi.init();

  // toggle RST low to reset; CS low so it'll listen to us
  _spi.CS_LOW();
  if (_rst != -1) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
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
  DC_HIGH();
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
  DC_HIGH();
  _spi.CS_LOW();

  spiwrite(color >> 8);
  spiwrite(color);

  _spi.CS_HIGH();
  _spi.END_TRANSACTION();
}


void Adafruit_ST77xx::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  fillRect(x, y, 1, h, color);
}


void Adafruit_ST77xx::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  fillRect(x, y, w, 1, color);
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

  _spi.BEGIN_TRANSACTION();

  DC_HIGH();
  _spi.CS_LOW();
#if 1
  color = ((color & 0xff) << 8) | (color >> 8);
  // about 4x speedup over byte-by-byte transfers
  uint32_t total = w*h;
  uint16_t buf[128];
  // XXX: need to fill the buf after transfer because SPI.transfer is overwriting it with the response.
  for (; total > 128; total -= 128)
  {
    for (int i = 0; i < 128; ++i)
    {
      buf[i] = color;
    }
    _spi.write(reinterpret_cast<uint8_t*>(buf), sizeof(buf));
  }
  for (int i = 0; i < total; ++i)
  {
    buf[i] = color;
  }
  _spi.write(reinterpret_cast<uint8_t*>(buf), sizeof(buf[0])*total);
#else
  uint8_t const hi = color >> 8;
  uint8_t const lo = color;
  for (int32_t i = w*h; i>0; --i) {
    spiwrite(hi);
    spiwrite(lo);
  }
#endif
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

void Adafruit_ST77xx::DC_HIGH() const {
#if defined(USE_FAST_IO)
  *dcport |= dcpinmask;
#else
  digitalWrite(_dc, HIGH);
#endif
}

void Adafruit_ST77xx::DC_LOW() const {
#if defined(USE_FAST_IO)
  *dcport &= ~dcpinmask;
#else
  digitalWrite(_dc, LOW);
#endif
}
