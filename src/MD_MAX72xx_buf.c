/*
MD_MAX72xx - Library for using a MAX7219/7221 LED matrix controller

See header file for comments
This file contains methods that act on display buffers.

Copyright (C) 2012-13 Marco Colli. All rights reserved.

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
#ifndef __MBED__
//#include <Arduino.h>
#endif

#include <memory.h>

#include "MD_MAX72xx.h"
#include "MD_MAX72xx_lib.h"

/**
 * \file
 * \brief Implements buffer related methods
 */

bool MD_MAX72XX_clear2(MD_MAX72XX_t *m,uint8_t buf)
{
  if (buf > LAST_BUFFER)
    return(false);

  memset(m->_matrix[buf].dig, 0, sizeof(m->_matrix[buf].dig));
  m->_matrix[buf].changed = ALL_CHANGED;

  if (m->_updateEnabled) MD_MAX72XX_flushBuffer(m,buf);

  return(true);
}

uint8_t MD_MAX72XX_bitReverse(uint8_t b)
// Reverse the order of bits within a byte.
// Returns the reversed byte value.
{
  b = ((b & 0xf0) >>  4) | ((b & 0x0f) << 4);
  b = ((b & 0xcc) >>  2) | ((b & 0x33) << 2);
  b = ((b & 0xaa) >>  1) | ((b & 0x55) << 1);

  return(b);
}

bool MD_MAX72XX_copyColumn(MD_MAX72XX_t *m,uint8_t buf, uint8_t cSrc, uint8_t cDest)
{
  if (m->_hwDigRows) return(MD_MAX72XX_copyC(m,buf, cSrc, cDest));
  else return(MD_MAX72XX_copyR(m,buf, cSrc, cDest));
}

bool MD_MAX72XX_copyRow(MD_MAX72XX_t *m,uint8_t buf, uint8_t rSrc, uint8_t rDest)
{
  if (m->_hwDigRows) return(MD_MAX72XX_copyR(m,buf, rSrc, rDest));
  else return(MD_MAX72XX_copyC(m,buf, rSrc, rDest));
}

bool MD_MAX72XX_copyC(MD_MAX72XX_t *m,uint8_t buf, uint8_t cSrc, uint8_t cDest)
// Src and Dest are in pixel coordinates.
// if we are just copying rows there is no need to repackage any data
{
  uint8_t maskSrc = 1 << HW_COL(cSrc);  // which column/row of bits is the column data

  if (m->_hwDigRows) { PRINT("\ncopyCol: (", buf); }
  else { PRINT("\ncopyRow: (", buf); }
  PRINT(", ", cSrc);
  PRINT(", ", cDest);
  PRINTS(") ");

  if ((buf > LAST_BUFFER) || (cSrc >= COL_SIZE) || (cDest >= COL_SIZE))
    return(false);

  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
      if (m->_matrix[buf].dig[i] & maskSrc)
        bitSet(m->_matrix[buf].dig[i], HW_COL(cDest));
    else
        bitClear(m->_matrix[buf].dig[i], HW_COL(cDest));
  }

  m->_matrix[buf].changed = ALL_CHANGED;

  if (m->_updateEnabled) MD_MAX72XX_flushBuffer(m,buf);

  return(true);
}

bool MD_MAX72XX_copyR(MD_MAX72XX_t *m,uint8_t buf, uint8_t rSrc, uint8_t rDest)
// Src and Dest are in pixel coordinates.
// if we are just copying digits there is no need to repackage any data
{
  if (m->_hwDigRows) { PRINT("\ncopyRow: (", buf); }
  else { PRINT("\ncopyColumn: (", buf); }
  PRINT(", ", rSrc);
  PRINT(", ", rDest);
  PRINTS(") ");

  if ((buf > LAST_BUFFER) || (rSrc >= ROW_SIZE) || (rDest >= ROW_SIZE))
    return(false);

  m->_matrix[buf].dig[HW_ROW(rDest)] = m->_matrix[buf].dig[HW_ROW(rSrc)];
  bitSet(m->_matrix[buf].changed, HW_ROW(rDest));

  if (m->_updateEnabled) MD_MAX72XX_flushBuffer(m,buf);

  return(true);
}

uint8_t MD_MAX72XX_getColumn(MD_MAX72XX_t *m,uint8_t buf, uint8_t c)
{
  if (m->_hwDigRows) return(MD_MAX72XX_getC(m,buf, c));
  else return(MD_MAX72XX_getR(m,buf, c));
}

uint8_t MD_MAX72XX_getRow(MD_MAX72XX_t *m,uint8_t buf, uint8_t r)
{
  if (m->_hwDigRows) return(MD_MAX72XX_getR(m,buf, r));
  else return(MD_MAX72XX_getC(m,buf, r));
}

uint8_t MD_MAX72XX_getC(MD_MAX72XX_t *m,uint8_t buf, uint8_t c)
// c is in pixel coordinates and the return value must be in pixel coordinate order
{
  uint8_t mask = 1 << HW_COL(c);  // which column/row of bits is the column data
  uint8_t value = 0;        // assembles data to be returned to caller

  if (m->_hwDigRows) { PRINT("\ngetCol: (", buf); }
  else { PRINT("\ngetRow: (", buf); }
  PRINT(", ", c);
  PRINTS(") ");

  if ((buf > LAST_BUFFER) || (c >= COL_SIZE))
    return(0);

  PRINTX("mask 0x", mask);

  // for each digit data, pull out the column/row bit and place
  // it in value. The loop creates the data in pixel coordinate order as it goes.
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
      if (m->_matrix[buf].dig[HW_ROW(i)] & mask)
        bitSet(value, i);
  }

  PRINTX(" value 0x", value);

  return(value);
}

uint8_t MD_MAX72XX_getR(MD_MAX72XX_t *m,uint8_t buf, uint8_t r)
// r is in pixel coordinates for this buffer
// returned value is in pixel coordinates
{
  if (m->_hwDigRows) { PRINT("\ngetRow: (", buf); }
  else { PRINT("\ngetCol: (", buf); }
  PRINT(", ", r);
  PRINTS(") ");

  if ((buf > LAST_BUFFER) || (r >= ROW_SIZE))
    return(0);

  uint8_t value = m->_hwRevCols ? MD_MAX72XX_bitReverse(m->_matrix[buf].dig[HW_ROW(r)]) : m->_matrix[buf].dig[HW_ROW(r)];

  PRINTX("0x", value);

  return(value);
}

bool MD_MAX72XX_setColumn(MD_MAX72XX_t *m,uint8_t buf, uint8_t c, uint8_t value)
{
  if (m->_hwDigRows) return(MD_MAX72XX_setC(m,buf, c, value));
  else return(MD_MAX72XX_setR(m,buf, c, value));
}

bool MD_MAX72XX_setRow1(MD_MAX72XX_t *m,uint8_t buf, uint8_t r, uint8_t value)
{
  if (m->_hwDigRows) return(MD_MAX72XX_setR(m,buf, r, value));
  else return(MD_MAX72XX_setC(m,buf, r, value));
}

bool MD_MAX72XX_setC(MD_MAX72XX_t *m,uint8_t buf, uint8_t c, uint8_t value)
// c and value are in pixel coordinate order
{
  if (m->_hwDigRows) { PRINT("\nsetCol: (", buf); }
  else { PRINT("\nsetRow: (", buf); }
  PRINT(", ", c);
  PRINTX(") 0x", value);

  if ((buf > LAST_BUFFER) || (c >= COL_SIZE))
    return(false);

  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
      if (value & (1 << i))   // mask off next column/row value passed in and set it in the dig buffer
        bitSet(m->_matrix[buf].dig[HW_ROW(i)], HW_COL(c));
      else
        bitClear(m->_matrix[buf].dig[HW_ROW(i)], HW_COL(c));
  }
  m->_matrix[buf].changed = ALL_CHANGED;

  if (m->_updateEnabled) MD_MAX72XX_flushBuffer(m,buf);

  return(true);
}

bool MD_MAX72XX_setR(MD_MAX72XX_t *m,uint8_t buf, uint8_t r, uint8_t value)
// r and value are in pixel coordinates
{
  if (m->_hwDigRows) { PRINT("\nsetRow: (", buf); }
  else { PRINT("\nsetCol: (", buf); }
  PRINT(", ", r);
  PRINTX(") 0x", value);

  if ((buf > LAST_BUFFER) || (r >= ROW_SIZE))
    return(false);

  m->_matrix[buf].dig[HW_ROW(r)] = m->_hwRevCols ? MD_MAX72XX_bitReverse(value) : value;
  bitSet(m->_matrix[buf].changed, HW_ROW(r));

  if (m->_updateEnabled) MD_MAX72XX_flushBuffer(m,buf);

  return(true);
}

bool MD_MAX72XX_transform2(MD_MAX72XX_t *m,uint8_t buf, enum transformType_t ttype)
{
  if (buf > LAST_BUFFER)
    return(false);

  if (!MD_MAX72XX_transformBuffer(m,buf, ttype))
    return(false);

  if (m->_updateEnabled) MD_MAX72XX_flushBuffer(m,buf);

  return(true);
}

bool MD_MAX72XX_transformBuffer(MD_MAX72XX_t *m,uint8_t buf, enum transformType_t ttype)
{
  uint8_t t[ROW_SIZE];

  switch (ttype)
  {
  //--------------
    case TSL: // Transform Shift Left one pixel element
      if (m->_hwDigRows)
      {
        for (uint8_t i=0; i<ROW_SIZE; i++)
        {
          if (m->_hwRevCols)
            m->_matrix[buf].dig[i] >>= 1;
          else
            m->_matrix[buf].dig[i] <<= 1;
        }
      }
      else
      {
        for (uint8_t i=ROW_SIZE; i>0; --i)
          m->_matrix[buf].dig[i] = m->_matrix[buf].dig[i-1];
      }
      break;

  //--------------
  case TSR: // Transform Shift Right one pixel element
      if (m->_hwDigRows)
      {
        for (uint8_t i=0; i<ROW_SIZE; i++)
        {
          if (m->_hwRevCols)
            m->_matrix[buf].dig[i] <<= 1;
          else
            m->_matrix[buf].dig[i] >>= 1;
        }
      }
      else
      {
        for (uint8_t i=0; i<ROW_SIZE-1; i++)
          m->_matrix[buf].dig[i] = m->_matrix[buf].dig[i+1];
      }
    break;

  //--------------
    case TSU: // Transform Shift Up one pixel element
    if (m->_wrapAround)  // save the first row or a zero row
      t[0] = MD_MAX72XX_getRow(m,buf, 0);
    else
      t[0] = 0;

    if (m->_hwDigRows)
    {
      for (uint8_t i=0; i<ROW_SIZE-1; i++)
          MD_MAX72XX_copyRow(m,buf, i+1, i);
    }
    else
    {
      for (int8_t i=ROW_SIZE-1; i>=0; i--)
        m->_matrix[buf].dig[i] <<= 1;
    }
    MD_MAX72XX_setRow1(m,buf, ROW_SIZE-1, t[0]);
    break;

  //--------------
    case TSD: // Transform Shift Down one pixel element
    if (m->_wrapAround)  // save the last row or a zero row
      t[0] = MD_MAX72XX_getRow(m,buf, ROW_SIZE-1);
    else
      t[0] = 0;

    if (m->_hwDigRows)
    {
      for (uint8_t i=ROW_SIZE; i>0; --i)
          MD_MAX72XX_copyRow(m,buf, i-1, i);
    }
    else
    {
      for (uint8_t i=0; i<ROW_SIZE; i++)
        m->_matrix[buf].dig[i] >>= 1;
    }
    MD_MAX72XX_setRow1(m,buf, 0, t[0]);
    break;

  //--------------
  case TFLR: // Transform Flip Left to Right
    if (m->_hwDigRows)
    {
      for (uint8_t i=0; i<ROW_SIZE; i++)
        m->_matrix[buf].dig[i] = MD_MAX72XX_bitReverse(m->_matrix[buf].dig[i]);
    }
    else  // really a TFUD
    {
      for (uint8_t i=0; i<ROW_SIZE/2; i++)
      {
        uint8_t	tt = m->_matrix[buf].dig[i];
        m->_matrix[buf].dig[i] = m->_matrix[buf].dig[ROW_SIZE-i-1];
        m->_matrix[buf].dig[ROW_SIZE-i-1] = tt;
      }
    }
    break;

  //--------------
  case TFUD: // Transform Flip Up to Down
    if (m->_hwDigRows)
    {
      for (uint8_t i=0; i<ROW_SIZE/2; i++)
      {
        uint8_t	tt = m->_matrix[buf].dig[i];
        m->_matrix[buf].dig[i] = m->_matrix[buf].dig[ROW_SIZE-i-1];
        m->_matrix[buf].dig[ROW_SIZE-i-1] = tt;
      }
    }
    else    // really a TFLR
    {
      for (uint8_t i=0; i<ROW_SIZE; i++)
        m->_matrix[buf].dig[i] = MD_MAX72XX_bitReverse(m->_matrix[buf].dig[i]);
    }
    break;

  //--------------
  case TRC: // Transform Rotate Clockwise
    for (uint8_t i=0; i<ROW_SIZE; i++)
      t[i] = MD_MAX72XX_getColumn(m,buf, COL_SIZE-1-i);

    for (uint8_t i=0; i<ROW_SIZE; i++)
        MD_MAX72XX_setRow1(m,buf, i, t[i]);
    break;

  //--------------
  case TINV: // Transform INVert
    for (uint8_t i=0; i<ROW_SIZE; i++)
      m->_matrix[buf].dig[i] = ~m->_matrix[buf].dig[i];
    break;

    default:
      return(false);
  }

  m->_matrix[buf].changed = ALL_CHANGED;

  return(true);
}
