/*
MD_MAX72xx - Library for using a MAX7219/7221 LED matrix controller

See header file for comments

This file contains class and hardware related methods.

Copyright (C) 2012-14 Marco Colli. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */
#ifdef __MBED__
//#include "mbed.h"
#else
//#include <Arduino.h>
//#include <SPI.h>
#endif

#include "MD_MAX72xx.h"
#include "MD_MAX72xx_lib.h"

/**
 * \file
 * \brief Implements class definition and general methods
 */

void MD_MAX72XX_constructor(MD_MAX72XX_t *m,enum moduleType_t mod, uint8_t dataPin, uint8_t clkPin, uint8_t csPin, uint8_t numDevices)
//_dataPin(dataPin), _clkPin(clkPin), _csPin(csPin),
//_hardwareSPI(false), _maxDevices(numDevices), _updateEnabled(true)
//#ifdef __MBED__
//, _spi((PinName)dataPin, NC, (PinName)clkPin),
//_cs((PinName)csPin)
//#endif
{
    m->_dataPin = dataPin;
    m->_clkPin = clkPin;
    m->_csPin = csPin;
    m->_hardwareSPI = false;
    m->_maxDevices = numDevices;
    m->_updateEnabled = true;
    MD_MAX72XX_setModuleParameters(m,mod);
}

void MD_MAX72XX_constructor2(MD_MAX72XX_t *m,enum moduleType_t mod, uint8_t csPin, uint8_t numDevices)
//_dataPin(0), _clkPin(0), _csPin(csPin),
//_hardwareSPI(true), _maxDevices(numDevices), _updateEnabled(true)
//#ifdef __MBED__
//, _spi(SPI_MOSI, NC, SPI_SCK),
//_cs((PinName)csPin)
//#endif
{
    m->_dataPin = 0;
    m->_clkPin = 0;
    m->_csPin = csPin;
    m->_hardwareSPI = true;
    m->_maxDevices = numDevices;
    m->_updateEnabled = true;
    MD_MAX72XX_setModuleParameters(m,mod);
}

void MD_MAX72XX_setModuleParameters(MD_MAX72XX_t *m,enum moduleType_t mod)
// Combinations not listed as tested have *probably* not 
// been tested and may not operate correctly.
{
  m->_mod = mod;
  switch (m->_mod)
  {
    case DR0CR0RR0_HW: m->_hwDigRows = false; m->_hwRevCols = false;  m->_hwRevRows = false; break;
    case DR0CR0RR1_HW: m->_hwDigRows = false; m->_hwRevCols = false;  m->_hwRevRows = true;  break;
    case DR0CR1RR0_HW: // same as GENERIC_HW, tested MC 9 March 2014
    case GENERIC_HW:   m->_hwDigRows = false; m->_hwRevCols = true;  m->_hwRevRows = false;  break;
    case DR0CR1RR1_HW: m->_hwDigRows = false; m->_hwRevCols = true;  m->_hwRevRows = true;   break;
    case DR1CR0RR0_HW: // same as FC16_HW, tested MC 23 Feb 2015
    case FC16_HW:      m->_hwDigRows = true;  m->_hwRevCols = false;  m->_hwRevRows = false; break;
    case DR1CR0RR1_HW: m->_hwDigRows = true;  m->_hwRevCols = false;  m->_hwRevRows = true;  break;
    case DR1CR1RR0_HW: // same as PAROLA_HW, tested MC 8 March 2014
    case PAROLA_HW:    m->_hwDigRows = true;  m->_hwRevCols = true;  m->_hwRevRows = false;  break;
    case DR1CR1RR1_HW: // same as ICSTATION_HW, tested MC 9 March 2014
    case ICSTATION_HW: m->_hwDigRows = true;  m->_hwRevCols = true;  m->_hwRevRows = true;   break;
  }
}

void MD_MAX72XX_begin(MD_MAX72XX_t *m)
{
  // initialize the SPI interface
  if (m->_hardwareSPI)
  {
#ifndef __MBED__
    PRINTS("\nHardware SPI");
//    SPI.begin();
#endif
  }
  else
  {
#ifndef __MBED__
    PRINTS("\nBitBang SPI")
//    pinMode(m->_dataPin, OUTPUT);
//    pinMode(m->_clkPin, OUTPUT);
#endif
  }

#ifndef __MBED__
  // initialize our preferred CS pin (could be same as SS)
//  pinMode(m->_csPin, OUTPUT);
//  digitalWrite(m->_csPin, HIGH);
#else
    m->_cs = 1;
#endif

  // object memory and internals
    MD_MAX72XX_setShiftDataInCallback(m,NULL);
    MD_MAX72XX_setShiftDataOutCallback(m,NULL);

    m->_matrix = (deviceInfo_t *)malloc(sizeof(deviceInfo_t) * m->_maxDevices);
    m->_spiData = (uint8_t *)malloc(SPI_DATA_SIZE);

#if USE_LOCAL_FONT
    MD_MAX72XX_setFont(m,_sysfont);
#endif // INCLUDE_LOCAL_FONT

  // Initialize the display devices. On initial power-up
  // - all control registers are reset,
  // - scan limit is set to one digit (row/col or LED),
  // - Decoding mode is off,
  // - intensity is set to the minimum,
  // - the display is blanked, and
  // - the MAX7219/MAX7221 is shut down.
  // The devices need to be set to our library defaults prior using the
  // display modules.
    MD_MAX72XX_control1(m,TEST, OFF);                   // no test
    MD_MAX72XX_control1(m,SCANLIMIT, ROW_SIZE-1);       // scan limit is set to max on startup
    MD_MAX72XX_control1(m,INTENSITY, MAX_INTENSITY/2);  // set intensity to a reasonable value
    MD_MAX72XX_control1(m,DECODE, OFF);                 // ensure no decoding (warm boot potential issue)
    MD_MAX72XX_clear1(m);
    MD_MAX72XX_control1(m,SHUTDOWN, OFF);               // take the modules out of shutdown mode
}

void MD_MAX72XX_destructor(MD_MAX72XX_t *m)
{
#ifndef __MBED__  
//  if (_hardwareSPI) SPI.end();  // reset SPI mode
#endif

  free(m->_matrix);
  free(m->_spiData);
}

void MD_MAX72XX_controlHardware(MD_MAX72XX_t *m,uint8_t dev, enum controlRequest_t mode, int value)
// control command is for the devices, translate internal request to device bytes
// into the transmission buffer
{
  uint8_t opcode = OP_NOOP;
  uint8_t param = 0;

  // work out data to write
  switch (mode)
  {
    case SHUTDOWN:
      opcode = OP_SHUTDOWN;
      param = (value == OFF ? 1 : 0);
      break;

    case SCANLIMIT:
      opcode = OP_SCANLIMIT;
      param = (value > MAX_SCANLIMIT ? MAX_SCANLIMIT : value);
      break;

    case INTENSITY:
      opcode = OP_INTENSITY;
      param = (value > MAX_INTENSITY ? MAX_INTENSITY : value);
      break;

    case DECODE:
      opcode = OP_DECODEMODE;
      param = (value == OFF ? 0 : 0xff);
      break;

    case TEST:
      opcode = OP_DISPLAYTEST;
      param = (value == OFF ? 0 : 1);
      break;

    default:
      return;
  }

  // put our device data into the buffer
  m->_spiData[SPI_OFFSET(dev, 0)] = opcode;
  m->_spiData[SPI_OFFSET(dev, 1)] = param;
}

void MD_MAX72XX_controlLibrary(MD_MAX72XX_t *m,enum controlRequest_t mode, int value)
// control command was internal, set required parameters
{
  switch (mode)
  {
    case UPDATE:
      m->_updateEnabled = (value == ON);
    if (m->_updateEnabled) MD_MAX72XX_flushBufferAll(m);
      break;

    case WRAPAROUND:
      m->_wrapAround = (value == ON);
      break;

    default:
      break;
  }
}

bool MD_MAX72XX_control2(MD_MAX72XX_t *m,uint8_t startDev, uint8_t endDev, enum controlRequest_t mode, int value)
{
  if (endDev < startDev) return(false);

  if (mode < UPDATE)  // device based control
  {
    MD_MAX72XX_spiClearBuffer(m);
    for (uint8_t i = startDev; i <= endDev; i++)
        MD_MAX72XX_controlHardware(m,i, mode, value);
    MD_MAX72XX_spiSend(m);
  }
  else                // internal control function, doesn't relate to specific device
  {
      MD_MAX72XX_controlLibrary(m,mode, value);
  }

  return(true);
}

bool MD_MAX72XX_control(MD_MAX72XX_t *m,uint8_t buf, enum controlRequest_t mode, int value)
// dev is zero based and needs adjustment if used
{
  if (buf > LAST_BUFFER) return(false);

  if (mode < UPDATE)  // device based control
  {
      MD_MAX72XX_spiClearBuffer(m);
      MD_MAX72XX_controlHardware(m,buf, mode, value);
      MD_MAX72XX_spiSend(m);
  }
  else                // internal control function, doesn't relate to specific device
  {
      MD_MAX72XX_controlLibrary(m,mode, value);
  }

  return(true);
}

void MD_MAX72XX_flushBufferAll(MD_MAX72XX_t *m)
// Only one data byte is sent to a device, so if there are many changes, it is more
// efficient to send a data byte all devices at the same time, substantially cutting
// the number of communication messages required.
{
  for (uint8_t i=0; i<ROW_SIZE; i++)  // all data rows
  {
    bool bChange = false; // set to true if we detected a change

    MD_MAX72XX_spiClearBuffer(m);

    for (uint8_t dev = FIRST_BUFFER; dev <= LAST_BUFFER; dev++)	// all devices
    {
      if (bitRead(m->_matrix[dev].changed, i))
      {
        // put our device data into the buffer
        m->_spiData[SPI_OFFSET(dev, 0)] = OP_DIGIT0+i;
        m->_spiData[SPI_OFFSET(dev, 1)] = m->_matrix[dev].dig[i];
        bChange = true;
      }
    }

    if (bChange)
        MD_MAX72XX_spiSend(m);
  }

  // mark everything as cleared
  for (uint8_t dev = FIRST_BUFFER; dev <= LAST_BUFFER; dev++)
    m->_matrix[dev].changed = ALL_CLEAR;
}

void MD_MAX72XX_flushBuffer(MD_MAX72XX_t *m,uint8_t buf)
// Use this function when the changes are limited to one device only.
// Address passed is a buffer address
{
  PRINT("\nflushBuf: ", buf);
  PRINTS(" r");

  if (buf > LAST_BUFFER)
    return;

  for (uint8_t i = 0; i < ROW_SIZE; i++)
  {
    if (bitRead(m->_matrix[buf].changed, i))
    {
      PRINT("", i);
      MD_MAX72XX_spiClearBuffer(m);

      // put our device data into the buffer
      m->_spiData[SPI_OFFSET(buf, 0)] = OP_DIGIT0+i;
      m->_spiData[SPI_OFFSET(buf, 1)] = m->_matrix[buf].dig[i];

      MD_MAX72XX_spiSend(m);
    }
  }
  m->_matrix[buf].changed = ALL_CLEAR;
}

void MD_MAX72XX_spiClearBuffer(MD_MAX72XX_t *m)
// Clear out the spi data array
{
  memset(m->_spiData, OP_NOOP, SPI_DATA_SIZE);
}
