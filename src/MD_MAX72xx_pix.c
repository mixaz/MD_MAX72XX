/*
MD_MAX72xx - Library for using a MAX7219/7221 LED matrix controller

See header file for comments

This file contains methods that act on the matrix as a pixel field,
generally only acting on the visible device range of the buffered
device field (ie, the physical pixel matrix).

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
#ifndef __MBED__
//#include <Arduino.h>
#endif

#include <memory.h>

#include "MD_MAX72xx.h"
#include "MD_MAX72xx_lib.h"

/**
 * \file
 * \brief Implements pixel related methods
 */

void MD_MAX72XX_clear(MD_MAX72XX_t *m,uint8_t startDev, uint8_t endDev)
{
  if (endDev < startDev) return;

  for (uint8_t buf = startDev; buf <= endDev; buf++)
  {
    memset(m->_matrix[buf].dig, 0, sizeof(m->_matrix[buf].dig));
    m->_matrix[buf].changed = ALL_CHANGED;
  }

  if (m->_updateEnabled) MD_MAX72XX_flushBufferAll(m);
}

bool MD_MAX72XX_getBuffer(MD_MAX72XX_t *m,uint16_t col, uint8_t size, uint8_t *pd)
{
  if ((col >= MD_MAX72XX_getColumnCount(m)) || (pd == NULL))
    return(false);

  for (uint8_t i=0; i<size; i++)
    *pd++ = MD_MAX72XX_getColumn1(m,col--);

  return(true);
}

bool MD_MAX72XX_setBuffer(MD_MAX72XX_t *m,uint16_t col, uint8_t size, uint8_t *pd)
{
  bool b = m->_updateEnabled;

  if ((col >= MD_MAX72XX_getColumnCount(m)) || (pd == NULL))
    return(false);

  m->_updateEnabled = false;
  for (uint8_t i=0; i<size; i++)
      MD_MAX72XX_setColumn2(m,col--, *pd++);
  m->_updateEnabled = b;

  if (m->_updateEnabled) MD_MAX72XX_flushBufferAll(m);

  return(true);
}

bool MD_MAX72XX_getPoint(MD_MAX72XX_t *m,uint8_t r, uint16_t c)
{
  uint8_t buf = c/COL_SIZE;

  c %= COL_SIZE;
  PRINT("\ngetPoint: (", buf);
  PRINT(", ", r);
  PRINT(", ", c);
  PRINTS(")");

  if ((buf > LAST_BUFFER) || (r >= ROW_SIZE) || (c >= COL_SIZE))
    return(false);

  if (m->_hwDigRows)
    return(bitRead(m->_matrix[buf].dig[HW_ROW(r)], HW_COL(c)) == 1);
  else
    return(bitRead(m->_matrix[buf].dig[HW_ROW(c)], HW_COL(r)) == 1);
}

bool MD_MAX72XX_setPoint(MD_MAX72XX_t *m,uint8_t r, uint16_t c, bool state)
{
  uint8_t buf = c/COL_SIZE;
  c %= COL_SIZE;

  PRINT("\nsetPoint: (", buf);
  PRINT(", ", r);
  PRINT(", ", c);
  PRINT(") = ", state?1:0);

  if ((buf > LAST_BUFFER) || (r >= ROW_SIZE) || (c >= COL_SIZE))
    return(false);

  if (state)
  {
    if (m->_hwDigRows)
      bitSet(m->_matrix[buf].dig[HW_ROW(r)], HW_COL(c));
    else
      bitSet(m->_matrix[buf].dig[HW_ROW(c)], HW_COL(r));
  }
  else
  {
    if (m->_hwDigRows)
      bitClear(m->_matrix[buf].dig[HW_ROW(r)], HW_COL(c));
    else
      bitClear(m->_matrix[buf].dig[HW_ROW(c)], HW_COL(r));
  }

  if (m->_hwDigRows)
    bitSet(m->_matrix[buf].changed, HW_ROW(r));
  else
    bitSet(m->_matrix[buf].changed, HW_ROW(c));

  if (m->_updateEnabled) MD_MAX72XX_flushBuffer(m,buf);

  return(true);
}

bool MD_MAX72XX_setRow(MD_MAX72XX_t *m,uint8_t startDev, uint8_t endDev, uint8_t r, uint8_t value)
{
  bool b = m->_updateEnabled;

  PRINT("\nsetRow: ", r);

  if ((r >= ROW_SIZE) || (endDev < startDev))
    return(false);

  m->_updateEnabled = false;
  for (uint8_t i = startDev; i <= endDev; i++)
      MD_MAX72XX_setRow1(m,i, r, value);
  m->_updateEnabled = b;

  if (m->_updateEnabled) MD_MAX72XX_flushBufferAll(m);

  return(true);
}

bool MD_MAX72XX_transform(MD_MAX72XX_t *m,uint8_t startDev, uint8_t endDev, enum transformType_t ttype)
{
 // uint8_t t[ROW_SIZE];
  uint8_t colData;
  bool b = m->_updateEnabled;

  if (endDev < startDev) return(false);

  m->_updateEnabled = false;

  switch (ttype)
  {
    case TSL: // Transform Shift Left one pixel element (with overflow)
    colData = 0;
    // if we can call the user function later then we don't need to do anything here
    // however, wraparound mode means we know the data so no need to request from the
    // callback at all - just save it for later
    if (m->_wrapAround)
      colData = MD_MAX72XX_getColumn1(m,((endDev+1)*COL_SIZE)-1);
    else if (m->_cbShiftDataOut != NULL)
      (*m->_cbShiftDataOut)(endDev, ttype, MD_MAX72XX_getColumn1(m,((endDev+1)*COL_SIZE)-1));

    // shift all the buffers along
    for (int8_t buf = endDev; buf >= startDev; --buf)
    {
      MD_MAX72XX_transformBuffer(m,buf, ttype);
      // handle the boundary condition
      MD_MAX72XX_setColumn(m,buf, 0, MD_MAX72XX_getColumn(m,buf-1, COL_SIZE-1));
    }

    // if we have a callback function, now is the time to get the data if we are
    // not in wraparound mode
    if (m->_cbShiftDataIn != NULL && !m->_wrapAround)
      colData = (*m->_cbShiftDataIn)(startDev, ttype);

    MD_MAX72XX_setColumn2(m,(startDev*COL_SIZE), colData);
    break;

    case TSR: // Transform Shift Right one pixel element (with overflow)
    // if we can call the user function later then we don't need to do anything here
    // however, wraparound mode means we know the data so no need to request from the
    // callback at all - just save it for later.
    colData = 0;
    if (m->_wrapAround)
      colData = MD_MAX72XX_getColumn1(m,startDev*COL_SIZE);
    else if (m->_cbShiftDataOut != NULL)
      (*m->_cbShiftDataOut)(startDev, ttype, MD_MAX72XX_getColumn1(m,(startDev*COL_SIZE)));

    // shift all the buffers along
    for (uint8_t buf=startDev; buf<=endDev; buf++)
    {
      MD_MAX72XX_transformBuffer(m,buf, ttype);

      // handle the boundary condition
      MD_MAX72XX_setColumn(m,buf, COL_SIZE-1, MD_MAX72XX_getColumn(m,buf+1, 0));
    }

    // if we have a callback function, now is the time to get the data if we are
    // not in wraparound mode
    if (m->_cbShiftDataIn != NULL && !m->_wrapAround)
      colData = (*m->_cbShiftDataIn)(endDev, ttype);

    MD_MAX72XX_setColumn2(m,((endDev+1)*COL_SIZE)-1, colData);
    break;

    case TFLR: // Transform Flip Left to Right (use the whole field)
    // first reverse the device buffers end for end
    for (uint8_t buf = 0; buf < (endDev - startDev + 1)/2; buf++)
    {
      deviceInfo_t	t;

      t = m->_matrix[startDev + buf];
      m->_matrix[startDev + buf] = m->_matrix[endDev - buf];
      m->_matrix[endDev - buf] = t;
    }

    // now reverse the columns in each device
    for (uint8_t buf = startDev; buf <= endDev; buf++)
        MD_MAX72XX_transformBuffer(m,buf, ttype);
    break;

    // These next transformations work the same just by doing the individual devices
    case TSU:   // Transform Shift Up one pixel element
    case TSD:   // Transform Shift Down one pixel element
    case TFUD:  // Transform Flip Up to Down
    case TRC:   // Transform Rotate Clockwise
    case TINV:  // Transform INVert
    for (uint8_t buf = startDev; buf <= endDev; buf++)
        MD_MAX72XX_transformBuffer(m,buf, ttype);
    break;

    default:
      return(false);
  }

  m->_updateEnabled = b;

  if (m->_updateEnabled) MD_MAX72XX_flushBufferAll(m);

  return(true);
}